"""Verify preview state does not silently alter the research measurements."""
from pathlib import Path
import sys, unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
from observatory_3d import Lab

class ObservatoryChecks(unittest.TestCase):
    def test_symmetric_elevation(self):
        lab=Lab();lab.set_mode(0);lab.flip()
        self.assertLess(lab.error,1e-12)
    def test_display_gain_and_camera_do_not_change_ping(self):
        lab=Lab();lab.running=False;before=lab.raw.copy();count=lab.ping_number
        lab.gain=12;lab.orbit=1;lab.step(1)
        np.testing.assert_array_equal(before,lab.raw)
        self.assertEqual(lab.ping_number,count)
    def test_motion_changes_measurement(self):
        lab=Lab();before=lab.raw.copy();lab.position[0]+=.5;lab.ping()
        self.assertGreater(np.max(np.abs(lab.raw-before)),0)
    def test_speckle_preserves_raw(self):
        lab=Lab();before=lab.raw.copy();lab.noise=True;lab.ping()
        np.testing.assert_array_equal(before,lab.raw)
        self.assertFalse(np.array_equal(lab.raw,lab.image))
    def test_all_experiments_have_finite_returns(self):
        lab=Lab()
        for mode in range(4):
            lab.set_mode(mode)
            self.assertTrue(np.isfinite(lab.raw).all())
            self.assertGreater(lab.raw.max(),0)
if __name__=='__main__':unittest.main(verbosity=2)
