"""Independent ARIS sequence baseline using this project's sonar front end.

No gantry or gimbal motion enters estimation. Ground truth is loaded only after
estimation. No IMU is synthesized. This is a sonar-only planar adaptation, not
validation of the existing synthetic sonar-IMU experiment or general 3-D SLAM.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
import pandas as pd
from PIL import Image
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sonar
import slam


class ArisAxes:
    def __init__(self, row, angles):
        self.angles = np.asarray(angles)
        self.ranges = (float(row.SampleStartDelay) +
                       (np.arange(int(row.SamplesPerBeam)) + 0.5) *
                       float(row.SamplePeriod)) * 1e-6 * float(row.SoundSpeed) / 2
    def azimuth_axis_deg(self):
        return self.angles
    def range_axis_m(self):
        return self.ranges


def estimate(recording, angles_by_count, count):
    # Only FrameIndex, FrameTime and range calibration fields are retained.
    columns = ['FrameIndex', 'FrameTime', 'SamplesPerBeam', 'SampleStartDelay',
               'SamplePeriod', 'SoundSpeed']
    meta = pd.read_csv(recording / 'aris_frame_meta.csv', usecols=columns).set_index('FrameIndex')
    files = sorted((recording / 'aris_raw').glob('*.pgm'))
    if len(files) != len(meta):
        raise ValueError('Incomplete extraction: raw frame and metadata counts differ')
    indices = np.unique(np.linspace(0, len(files)-1, min(count, len(files))).round().astype(int))
    files = [files[i] for i in indices]
    ids = [int(f.stem) for f in files]
    times = meta.loc[ids, 'FrameTime'].to_numpy(dtype=float) * 1e-6
    if not np.all(np.diff(times) > 0):
        raise ValueError('Non-increasing frame timestamps')
    images, features, descriptors, axes_list = [], [], [], []
    for f in files:
        raw = np.asarray(Image.open(f), dtype=float)
        row = meta.loc[int(f.stem)]
        assert raw.shape[0] == int(row.SamplesPerBeam)
        axes = ArisAxes(row, angles_by_count[raw.shape[1]])
        image = raw.T.copy()  # native PGM is range x bearing; project uses bearing x range
        points, _ = slam.extract_features(image, axes, relative_threshold=0.025,
                                          max_features=100, exclusion_bins=(2, 4))
        images.append(image)
        features.append(points)
        descriptors.append(slam.scan_descriptor(image))
        axes_list.append(axes)
    graph = sonar.PlanarSlam(initial=(0, 0, 0))
    edges, rejected = [], []
    previous_motion = np.zeros(3)
    for i in range(1, len(files)):
        initial = previous_motion.copy()
        if i > 1:
            initial *= (times[i]-times[i-1])/(times[i-1]-times[i-2])
        measured, error, inliers = slam.icp_relative(features[i-1], features[i],
                                                   initial=initial, max_correspondence_m=0.60)
        accepted = inliers >= 5 and error < 0.22
        if accepted:
            graph.add_odometry(measured, sigma_translation=max(0.025, error),
                               sigma_yaw=math.radians(0.8))
            previous_motion = measured
        else:
            # Explicit hold on failure; do not fill from ground truth or synthetic IMU.
            graph.add_odometry((0,0,0), sigma_translation=10.0, sigma_yaw=math.pi)
            previous_motion = np.zeros(3)
            rejected.append(i)
        edges.append(dict(index=i, accepted=bool(accepted), inliers=int(inliers),
                          residual_m=float(error) if np.isfinite(error) else None))
    before = graph.poses().copy()
    loops = []
    for i in range(12, len(files)):
        candidate, similarity = slam.candidate_loop(descriptors, i, minimum_separation=12)
        if candidate is None or similarity < 0.91:
            continue
        initial = sonar.relative_pose_2d(graph.poses()[candidate], graph.poses()[i])
        if np.linalg.norm(initial[:2]) > 1.5:
            continue
        measured, error, inliers = slam.icp_relative(features[candidate], features[i],
                     initial=initial, max_correspondence_m=max(0.7, np.linalg.norm(initial[:2])+0.35))
        if inliers >= 6 and error < 0.18:
            graph.add_loop_closure(candidate, i, measured, sigma_translation=max(0.02,error),
                                  sigma_yaw=math.radians(0.6))
            loops.append(dict(first=candidate, second=i, similarity=float(similarity),
                              inliers=int(inliers), residual_m=float(error), measured_pose=np.asarray(measured).tolist()))
    graph.optimize(iterations=40, huber_delta=2.5)
    return dict(ids=ids, times=times-times[0], images=images, features=features,
                axes=axes_list, before=before, poses=graph.poses().copy(),
                edges=edges, loops=loops, rejected=rejected, input_files=files)


def reference(recording, ids):
    """Published gantry-to-AR3 mount and AR3-to-sonar lever arm.

    Same rotation order as dataset release_1_export.py: base R * Euler xyz.
    Beam horizontal plane assumed for evaluation; actual tilt/elevation are
    reported as model violations, not silently corrected from reference data.
    """
    meta = pd.read_csv(recording/'aris_frame_meta.csv').set_index('FrameIndex').loc[ids]
    crane = pd.read_csv(recording/'gantry.csv').set_index('aris_frame_idx').loc[ids]
    angles = meta[['SonarRoll','SonarTilt','SonarPan']].to_numpy()
    rotations = np.diag([-1.,1.,-1.])[None,:,:] @ Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
    positions = crane[['x','y','z']].to_numpy() + np.array([0.,0.174,-0.338])
    positions += np.einsum('nij,j->ni', rotations, [-0.108725,0.,0.])
    right = rotations[:,:,1]
    yaw = np.arctan2(right[:,1], right[:,0])
    return np.column_stack([positions[:,:2], yaw]), positions, angles


def rotate_xy(xy, yaw):
    c,s = np.cos(yaw),np.sin(yaw)
    return xy @ np.array([[c,-s],[s,c]]).T


def score(estimate_xy, truth_xy):
    # Proper SE(2) least-squares alignment, no reflection and no scale fitting.
    a,b = estimate_xy-estimate_xy.mean(0), truth_xy-truth_xy.mean(0)
    u,_,vt=np.linalg.svd(a.T @ b)
    rotation=vt.T @ u.T
    if np.linalg.det(rotation)<0:
        vt[-1]*=-1
        rotation=vt.T @ u.T
    aligned=a@rotation.T+truth_xy.mean(0)
    errors=np.linalg.norm(aligned-truth_xy,axis=1)
    return aligned, errors


def run(recording, angles, count, out):
    print('Estimating', recording.name, flush=True)
    result = estimate(recording, angles, count)
    # Reference file access begins here, after all matching and graph optimization.
    truth, positions, gimbal = reference(recording, result['ids'])
    poses=result['poses']
    initial_aligned=rotate_xy(poses[:,:2], truth[0,2])+truth[0,:2]
    aligned, errors=score(poses[:,:2], truth[:,:2])
    before_aligned, before_errors=score(result['before'][:,:2],truth[:,:2])
    for loop in result['loops']:
        reference_motion=np.asarray(sonar.relative_pose_2d(truth[loop['first']],truth[loop['second']]))
        difference=np.asarray(loop['measured_pose'])-reference_motion
        loop['reference_translation_error_m']=float(np.linalg.norm(difference[:2]))
        loop['reference_yaw_error_deg']=float(abs(np.degrees(np.arctan2(np.sin(difference[2]),np.cos(difference[2])))))
    stationary=np.sqrt(np.mean(np.sum((truth[:,:2]-truth[:,:2].mean(0))**2,axis=1)))
    estimated_length=float(np.linalg.norm(np.diff(poses[:,:2],axis=0),axis=1).sum())
    true_length=float(np.linalg.norm(np.diff(truth[:,:2],axis=0),axis=1).sum())
    # Adjacent selected-frame RPE; same local coordinate convention as estimation.
    relative_errors=[]
    for i in range(1,len(poses)):
        et=sonar.relative_pose_2d(poses[i-1],poses[i])
        gt=sonar.relative_pose_2d(truth[i-1],truth[i])
        relative_errors.append(np.linalg.norm(np.asarray(et[:2])-gt[:2]))
    metrics=dict(sequence=recording.name, frame_count=len(poses), duration_s=float(result['times'][-1]),
        frame_ids=result['ids'], accepted_pairs=len(result['edges'])-len(result['rejected']),
        rejected_pairs=len(result['rejected']), loop_constraints=len(result['loops']),
        ate_se2_rmse_m=float(np.sqrt(np.mean(errors**2))),
        before_loops_ate_se2_rmse_m=float(np.sqrt(np.mean(before_errors**2))),
        median_loop_translation_error_m=float(np.median([x['reference_translation_error_m'] for x in result['loops']])) if result['loops'] else None,
        median_loop_yaw_error_deg=float(np.median([x['reference_yaw_error_deg'] for x in result['loops']])) if result['loops'] else None,
        initial_alignment_rmse_m=float(np.sqrt(np.mean(np.sum((initial_aligned-truth[:,:2])**2,axis=1)))),
        stationary_se2_rmse_m=float(stationary),
        relative_translation_rmse_m=float(np.sqrt(np.mean(np.square(relative_errors)))),
        estimated_path_length_m=estimated_length, reference_path_length_m=true_length,
        reference_vertical_span_m=float(np.ptp(positions[:,2])),
        sonar_tilt_minmax_deg=[float(gimbal[:,1].min()),float(gimbal[:,1].max())],
        sonar_roll_minmax_deg=[float(gimbal[:,0].min()),float(gimbal[:,0].max())],
        beats_stationary_baseline=bool(np.sqrt(np.mean(errors**2)) < stationary),
        scope='Sonar-only planar adaptation; no IMU; real tilted sonar violates horizontal-plane model.')
    folder=out/recording.name;folder.mkdir()
    pd.DataFrame(dict(frame_id=result['ids'],time_s=result['times'],
        estimate_x=poses[:,0],estimate_y=poses[:,1],estimate_yaw=poses[:,2],
        reference_x=truth[:,0],reference_y=truth[:,1],reference_yaw=truth[:,2],
        before_x=result['before'][:,0],before_y=result['before'][:,1],before_yaw=result['before'][:,2],
        aligned_x=aligned[:,0],aligned_y=aligned[:,1],error_m=errors)).to_csv(folder/'trajectory.csv',index=False)
    (folder/'metrics.json').write_text(json.dumps(metrics,indent=2))
    (folder/'matching.json').write_text(json.dumps(dict(edges=result['edges'],loops=result['loops']),indent=2))
    fig,ax=plt.subplots(1,3,figsize=(16,4.8),constrained_layout=True)
    sample=len(poses)//2;axes=result['axes'][sample]
    ax[0].imshow(result['images'][sample],origin='lower',aspect='auto',cmap='gray',
                 extent=[axes.ranges[0],axes.ranges[-1],axes.angles[0],axes.angles[-1]],vmin=0,vmax=255)
    ax[0].set(title='Recorded ARIS frame (display values)',xlabel='Slant range (m)',ylabel='Bearing (deg)')
    txy=truth[:,:2]-truth[0,:2]
    ax[1].plot(*txy.T,'k-',label='Gantry + calibrated sonar offset')
    ax[1].plot(*(before_aligned-truth[0,:2]).T,color='#c74f24',linestyle='--',label='Before loops, SE(2) aligned')
    ax[1].plot(*(aligned-truth[0,:2]).T,color='#007c91',label='After loops, SE(2) aligned')
    ax[1].set(title='Horizontal trajectory; no scale fitting',xlabel='X (m)',ylabel='Y (m)')
    ax[1].axis('equal');ax[1].legend(fontsize=7)
    ax[2].plot(result['times'],errors,color='#c74f24')
    ax[2].set(title=f'Aligned position RMSE: {metrics["ate_se2_rmse_m"]:.3f} m',xlabel='Time (s)',ylabel='Error (m)')
    for a in ax[1:]:a.grid(alpha=0.2)
    fig.suptitle(f'DFKI real sonar baseline — {recording.name}; not a full sonar–IMU validation')
    fig.savefig(folder/'comparison.png',dpi=150);plt.close(fig)
    print(json.dumps(metrics),flush=True)
    return metrics


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--data',type=Path,required=True)
    parser.add_argument('--beam-table',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--frames',type=int,default=37)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    angles={int(k):v for k,v in json.loads(args.beam_table.read_text()).items()}
    recordings=sorted(p.parent for p in args.data.glob('*/gantry.csv'))
    if not recordings:raise ValueError('No released recordings found')
    metrics=[run(p,angles,args.frames,args.output) for p in recordings]
    (args.output/'summary.json').write_text(json.dumps(metrics,indent=2))
    provenance=dict(dataset='https://zenodo.org/records/13778485',
        tools='https://github.com/dfki-ric/uxo-dataset2024',
        estimator_source_sha256=hashlib.sha256((ROOT/'python/slam.py').read_bytes()).hexdigest(),
        benchmark_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        sampling=f'{args.frames} uniformly spaced frames over each complete sample recording',
        parameters='Fixed before reading trajectory errors; same feature and acceptance thresholds as synthetic demo.',
        differences='No IMU. Last accepted relative sonar motion initializes ICP; rejection holds pose with weak graph edge.',
        ground_truth='Gantry positions and gimbal readings used after estimation only; published mounting transform applied.',
        metric='Horizontal ATE after proper SE(2) fit without scale or reflection; also initial alignment and stationary baseline.',
        limitations=['Not a commonly used full SLAM benchmark; object-centric tank recordings.',
                     'Tilted sonar and elevation ambiguity violate planar model.',
                     'Raw 8-bit sonar display values are not calibrated linear intensity.',
                     'No synthetic IMU generated from reference trajectory.',
                     'Reference synchronization is supplied by dataset authors, not independently remeasured.',
                     'Accepted ICP correspondences and loop constraints are not necessarily correct.'])
    (args.output/'provenance.json').write_text(json.dumps(provenance,indent=2))

if __name__=='__main__': main()
