import unittest
import numpy as np
from admission import require_rigid_mount

class AdmissionTests(unittest.TestCase):
    def test_rigid_mount_with_translation(self):
        t=np.eye(4); t[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];t[:3,3]=[.15,0,.15]
        np.testing.assert_array_equal(require_rigid_mount(t), t)
    def test_published_shear_rejected_even_with_unit_determinant(self):
        t=np.eye(4);t[:3,:3]=[[1,1,0],[0,-1,0],[0,0,-1]]
        self.assertEqual(np.linalg.det(t[:3,:3]),1)
        with self.assertRaisesRegex(ValueError,'orthonormal'):require_rigid_mount(t)
    def test_reflection_and_nonfinite_rejected(self):
        t=np.eye(4);t[0,0]=-1
        with self.assertRaises(ValueError):require_rigid_mount(t)
        t=np.eye(4);t[0,3]=np.nan
        with self.assertRaises(ValueError):require_rigid_mount(t)
if __name__=='__main__':unittest.main(verbosity=2)
