import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from geometry import *
class GeometryChecks(unittest.TestCase):
 def test_tilted_full_3d_forward_prediction(self):
  r=Rotation.from_euler('xyz',[-.7,.12,.4]).as_matrix();origin=np.array([1,2,.3])
  local=np.array([[1,4,-2],[-1,3,1],[.6,5,.1]])
  observed=project(local@r.T+origin,origin,r)
  np.testing.assert_allclose(observed[:,0],np.linalg.norm(local,axis=1))
  np.testing.assert_allclose(observed[:,1],np.arctan2(local[:,0],local[:,1]))
  self.assertGreater(abs(observed[0,0]-np.linalg.norm(local[0,:2])),.1)
 def test_pose_recovery_surveyed_targets(self):
  points=np.array([[-1,4,-1],[1,4,1],[-.7,5,.8],[.8,3,-.5],[0,6,1.4],[1.4,5,-1.2]])
  truth=np.array([.1,-.2,.05,-.12,.03,.05])
  obs=project(points,truth[:3],Rotation.from_rotvec(truth[3:]).as_matrix())
  result=localize_surveyed_targets(points,obs,np.zeros(6))
  self.assertTrue(result['success']);self.assertEqual(result['rank'],6)
  np.testing.assert_allclose(result['pose'],truth,atol=1e-6)
 def test_elevation_ambiguity_is_preserved(self):
  np.testing.assert_allclose(project([[1,4,2]],[0,0,0],np.eye(3)),project([[1,4,-2]],[0,0,0],np.eye(3)))
 def test_reference_orientation_is_forbidden(self):
  with self.assertRaises(ValueError):measured_orientation_prior(np.zeros(6),np.eye(3),.01,'ground_truth')
 def test_off_axis_sign_and_inconclusive_cases(self):
  points=np.array([[-1,4,0],[1,4,0]]);b=np.arctan2(points[:,0],points[:,1])
  self.assertEqual(calibrate_bearing_sign(points,-b,.01,'surveyed_off_axis_target'),-1)
  with self.assertRaises(ValueError):calibrate_bearing_sign(points,[0,0],.01,'surveyed_off_axis_target')
if __name__=='__main__':unittest.main(verbosity=2)
