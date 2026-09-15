"""Independent 3-D fit to public off-axis sphere/sonar calibration observations.
Calibration-only mocap is legitimate here; no navigation test labels are used.
Extrinsics from this separate fixture must NOT be transferred to another mounting.
"""
import json,sys
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
root=ROOT/'data_external/sonar_extrinsics'
m=json.loads((root/'markerCoordData.json').read_text());g=json.loads((root/'motionCaptureData.json').read_text())
poses=np.array(g['aligned_sonarFixatorPose']);xyz=[];obs=[]
for k in range(1,5):
 p=np.einsum('nij,nj->ni',np.linalg.inv(poses),np.array(g[f'marker{k}GT']))[:,:3]
 xyz.append(p);o=np.array(m[f'marker{k}']);o[:,1]=np.radians(o[:,1]);obs.append(o)
xyz=np.stack(xyz,axis=1);obs=np.stack(obs,axis=1)
train=np.arange(8);validation=np.arange(8,12)
def predict(v,points):
 q=points@Rotation.from_rotvec(v[:3]).as_matrix().T+v[3:]
 return np.stack((np.linalg.norm(q,axis=-1),np.arctan2(q[...,1],q[...,0])),axis=-1),q
results=[]
for sign in [1,-1]:
 target=obs.copy();target[...,1]*=sign
 def fun(v):
  pred,q=predict(v,xyz[train]);d=pred-target[train];d[...,1]=np.arctan2(np.sin(d[...,1]),np.cos(d[...,1]))
  return np.r_[(d/np.array([.01,.005])).ravel(),np.minimum(q[...,0],0).ravel()/.01]
 rng=np.random.default_rng(222)
 starts=[np.zeros(6),np.r_[Rotation.from_euler('xyz',[180,0,90],degrees=True).as_rotvec(),[-.1,0,-.2]]]+[np.r_[Rotation.random(random_state=rng).as_rotvec(),np.zeros(3)] for _ in range(10)]
 fits=[least_squares(fun,v,max_nfev=500) for v in starts];best=min(fits,key=lambda x:np.sum(x.fun**2))
 row=dict(bearing_sign=sign,transform_fixture_to_sonar=np.eye(4).tolist(),train_board_indices=train.tolist(),validation_board_indices=validation.tolist())
 t=np.eye(4);t[:3,:3]=Rotation.from_rotvec(best.x[:3]).as_matrix();t[:3,3]=best.x[3:];row['transform_fixture_to_sonar']=t.tolist()
 sv=np.linalg.svd(best.jac,compute_uv=False);row['jacobian_singular_values']=sv.tolist()
 for name,indices in [('train',train),('validation',validation)]:
  pred,q=predict(best.x,xyz[indices]);d=pred-target[indices];d[...,1]=np.arctan2(np.sin(d[...,1]),np.cos(d[...,1]))
  row[name]=dict(range_rmse_m=float(np.sqrt(np.mean(d[...,0]**2))),bearing_rmse_deg=float(np.degrees(np.sqrt(np.mean(d[...,1]**2)))),targets_behind=int(np.sum(q[...,0]<=0)))
 results.append(row)
(HERE/'calibration_board_results.json').write_text(json.dumps(dict(source='https://github.com/hwgao1101/sonar_Extrinsics_Calibration',scope='Dedicated calibration fixture only; cannot transfer to ARIS or 2023 BlueROV without verified mounting identity',results=results),indent=2))
for r in results:print(r['bearing_sign'],r['train'],r['validation'])
