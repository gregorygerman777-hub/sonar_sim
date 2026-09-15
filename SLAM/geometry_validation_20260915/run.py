"""Off-axis calibration and new planar synthetic references, never physical proof."""
import sys, json, hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'python'),str(ROOT/'SLAM/research')]
import sonar, slam, slam_experiment
from admission import require_rigid_mount
from evaluate import estimate
from benchmark_public_uxo import score
HERE=Path(__file__).resolve().parent
OUT=ROOT/'output/geometry_validation_20260915'
OUT.mkdir(parents=True,exist_ok=True)

def calibration():
    sim=slam_experiment.build_simulator();rows=[]
    for yaw in [0.,.7]:
        axes=slam.pose_axes([0,0,yaw]);origin=np.array([2.,-1.,-.5])
        for x in [-1.,1.]:
            local=np.array([x,4.,0.]);world=origin+axes@local
            image=sim.render([sonar.make_sphere(world,.06,.9)],position=origin,axes=axes)
            b,r=np.unravel_index(np.argmax(image),image.shape)
            if image[b,r]<=0:raise RuntimeError('Calibration target not detected')
            observed=float(sim.azimuth_axis_deg()[b]);expected=float(np.degrees(np.arctan2(x,4.)))
            rows.append(dict(x_right_m=x,world_yaw_rad=yaw,expected_deg=expected,observed_deg=observed,error_deg=abs(observed-expected)))
    return dict(scope='simulator_only',positive_bearing_axis='sonar X-right',passed=all(r['error_deg']<=1 for r in rows),observations=rows)

def sequence(seed):
    rng=np.random.default_rng(seed);sim=slam_experiment.build_simulator()
    objects=[sonar.make_sphere((x,y,-1.),radius,reflectivity) for x,y,radius,reflectivity in
             zip(rng.uniform(-2,2,18),rng.uniform(-2,2,18),rng.uniform(.08,.16,18),rng.uniform(.5,1.,18))]
    angles=np.linspace(0,1.5*np.pi,81)
    truth=np.c_[4.8*np.cos(angles),4.8*np.sin(angles),angles+np.pi/2]
    positions=np.c_[truth[:,:2],np.full(81,-1.)]
    _,points,desc=slam_experiment.render_survey(sim,objects,truth,positions)
    # The estimator receives measurements/times only. Truth stays outside inputs.
    inputs=dict(ids=np.arange(81),times=np.arange(81)*.4,points=points,desc=desc)
    result=estimate(inputs,1);keys=result['keys'];reference=truth[keys,:2]
    _,before=score(result['poses'][keys,:2],reference)
    aligned,after=score(result['optimized'][:,:2],reference)
    stationary=np.sqrt(np.mean(np.sum((reference-reference.mean(0))**2,axis=1)))
    metrics=dict(seed=seed,frames=81,odometry_ate_m=float(np.sqrt(np.mean(before**2))),
       loop_ate_m=float(np.sqrt(np.mean(after**2))),stationary_ate_m=float(stationary),
       rejected_pairs=sum(not e['accepted'] for e in result['edges']),loops=len(result['loops']))
    np.savez(OUT/f'seed_{seed}.npz',truth=truth,odometry=result['poses'],optimized=result['optimized'],aligned=aligned)
    return metrics

if __name__=='__main__':
    results=dict(protocol_sha256=hashlib.sha256((HERE/'PROTOCOL.md').read_bytes()).hexdigest(),calibration=calibration())
    if not results['calibration']['passed']:raise RuntimeError('Off-axis calibration failed')
    results['sequences']=[sequence(s) for s in [91501,91502,91503]]
    # Supplied BlueROV T_BS, transcribed without silently repairing it.
    R=np.array([[1.,1.,0.],[0.,-1.,0.],[0.,0.,-1.]])
    mount=np.eye(4); mount[:3,:3]=R; mount[:3,3]=[.15,0,.15]
    try:
        require_rigid_mount(mount)
        mount_error=None
    except ValueError as exc:
        mount_error=str(exc)
    results['physical_admission']=dict(mount_error=mount_error, source='data_external/bluerov_uuv/TransMatrix.yaml:T_BS',
        orthogonality_error=float(np.linalg.norm(R.T@R-np.eye(3))),determinant=float(np.linalg.det(R)),
        admitted=False,reason='Mount rotation is not orthonormal; no independently surveyed physical bearing target available')
    (OUT/'results.json').write_text(json.dumps(results,indent=2))
    print(json.dumps(results,indent=2))
