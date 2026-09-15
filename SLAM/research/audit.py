"""Post-estimation metadata and calibration audit; never imported by estimator."""
from evaluate import *
from scipy.spatial.transform import Rotation

def audit_recording(rec):
 meta=pd.read_csv(rec/'aris_frame_meta.csv').set_index('FrameIndex');ids=meta.index.to_numpy()
 truth,positions,angles=reference(rec,ids)
 gantry=pd.read_csv(rec/'gantry.csv').set_index('aris_frame_idx').loc[ids]
 # Independent homogeneous chain from published quaternions and lever arm.
 mount=np.eye(4);mount[:3,:3]=Rotation.from_quat([0,1,0,0]).as_matrix();mount[:3,3]=[0,.174,-.338]
 sensor=np.eye(4);sensor[:3,:3]=Rotation.from_quat([.5,.5,.5,.5]).as_matrix();sensor[:3,3]=[-.108725,0,0]
 delta=[];orth=[]
 for i,(_,row) in enumerate(gantry.iterrows()):
  world=np.eye(4);world[:3,3]=row[['x','y','z']].to_numpy(float)
  gimbal=np.eye(4);gimbal[:3,:3]=Rotation.from_euler('xyz',angles[i],degrees=True).as_matrix()
  transform=world@mount@gimbal@sensor
  delta.append(np.linalg.norm(transform[:3,3]-positions[i]))
  right=transform[:3,0];orth.append(abs(wrap(np.arctan2(right[1],right[0])-truth[i,2])))
 timestamps=meta.FrameTime.to_numpy(float)*1e-6;dt=np.diff(timestamps)
 sound=meta.SoundSpeed.to_numpy(float)
 low=meta.SampleStartDelay.to_numpy(float)*sound*1e-6/2
 high=(meta.SampleStartDelay.to_numpy(float)+meta.SamplesPerBeam.to_numpy(float)*meta.SamplePeriod.to_numpy(float))*sound*1e-6/2
 return dict(sequence=rec.name,frames=len(ids),timestamp_strictly_increasing=bool(np.all(dt>0)),dt_min_median_max_s=np.quantile(dt,[0,.5,1]).tolist(),
  sound_speed_minmax_m_s=[float(sound.min()),float(sound.max())],range_window_minmax_m=[float(low.min()),float(high.max())],
  homogeneous_position_disagreement_m=float(max(delta)),homogeneous_yaw_disagreement_rad=float(max(orth)),
  pitch_minmax_deg=[float(angles[:,1].min()),float(angles[:,1].max())],
  zero_elevation_forward_horizontal_scale_minmax=[float(np.cos(np.radians(angles[:,1])).min()),float(np.cos(np.radians(angles[:,1])).max())],
  reference_sha256={n:hashlib.sha256((rec/n).read_bytes()).hexdigest() for n in ['gantry.csv','aris_frame_meta.csv']})

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--data',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
 results=[audit_recording(p.parent) for p in sorted(args.data.rglob('gantry.csv'))]
 args.output.write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
