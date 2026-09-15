"""3-D range/bearing factors; local axes x-right, y-forward, z-up.
No elevation=0 substitution. No reference orientation accepted as sensor input.
This module is not yet integrated into the planar C++ SLAM graph.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares

def rotation_checked(rotation):
 r=np.asarray(rotation,float)
 if r.shape!=(3,3) or not np.allclose(r.T@r,np.eye(3),atol=1e-8) or not np.isclose(np.linalg.det(r),1):raise ValueError('Expected a proper rotation')
 return r

def project(world_points,position,world_from_sonar):
 q=(np.asarray(world_points,float)-np.asarray(position,float))@rotation_checked(world_from_sonar)
 distance=np.linalg.norm(q,axis=-1)
 if np.any(distance<=0) or np.any(q[...,1]<=0):raise ValueError('Targets must be nonzero and in front of sonar')
 return np.stack((distance,np.arctan2(q[...,0],q[...,1])),axis=-1)

def range_bearing_factor(world_points,pose6,observations,sigma_range=.01,sigma_bearing=.002):
 # Rotation vector is a genuine state variable, not a reference pose input.
 rotation=Rotation.from_rotvec(np.asarray(pose6)[3:]).as_matrix()
 predicted=project(world_points,np.asarray(pose6)[:3],rotation)
 residual=predicted-np.asarray(observations)
 residual[...,1]=np.arctan2(np.sin(residual[...,1]),np.cos(residual[...,1]))
 return (residual/np.array([sigma_range,sigma_bearing])).ravel()

def measured_orientation_prior(pose6,measurement,sigma_rad,source):
 if source not in {'measured_imu','measured_encoder','surveyed_mount'}:raise ValueError('Orientation must come from a legitimate measured input, never reference/ground truth')
 if sigma_rad<=0:raise ValueError('Positive measured uncertainty required')
 measured=rotation_checked(measurement)
 estimate=Rotation.from_rotvec(np.asarray(pose6)[3:]).as_matrix()
 return Rotation.from_matrix(measured.T@estimate).as_rotvec()/sigma_rad

def localize_surveyed_targets(points,observations,initial):
 """Known-map localization control, NOT unknown-map SLAM."""
 result=least_squares(lambda p:range_bearing_factor(points,p,observations),initial,max_nfev=300)
 sv=np.linalg.svd(result.jac,compute_uv=False)
 return dict(pose=result.x,success=bool(result.success),jacobian_singular_values=sv,rank=int(np.sum(sv>sv[0]*1e-8)))

def calibrate_bearing_sign(target_xyz,recorded_bearing_rad,tolerance_rad,position_source):
 """Resolve sign using surveyed isolated off-axis targets, not path ATE.
Targets must be expressed in the independently measured sonar frame.
"""
 if position_source!='surveyed_off_axis_target':raise ValueError('Independent off-axis target survey required')
 p=np.asarray(target_xyz,float);b=np.asarray(recorded_bearing_rad,float)
 if p.ndim!=2 or p.shape[1]!=3 or len(p)<2 or len(b)!=len(p):raise ValueError('At least two target measurements required')
 if not(np.any(p[:,0]>0) and np.any(p[:,0]<0)):raise ValueError('Targets on both left and right required')
 expected=project(p,[0,0,0],np.eye(3))[:,1]
 if np.min(abs(expected))<3*tolerance_rad:raise ValueError('Targets too close to boresight')
 errors={s:np.abs(np.arctan2(np.sin(s*b-expected),np.cos(s*b-expected))) for s in [1,-1]}
 valid=[s for s,e in errors.items() if np.max(e)<=tolerance_rad]
 if len(valid)!=1:raise ValueError('Calibration inconclusive; do not choose lowest trajectory error')
 return valid[0]
