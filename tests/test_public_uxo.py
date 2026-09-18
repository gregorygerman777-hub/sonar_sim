"""Checks for physical bin mapping and trajectory scoring in real-data adapter."""
import sys
from pathlib import Path
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parents[1]/'python')]
import unittest
from types import SimpleNamespace
import numpy as np
from benchmark_public_uxo import ArisAxes, score, rotate_xy

class PublicSequenceChecks(unittest.TestCase):
    def test_microsecond_range_bin_centers(self):
        row=SimpleNamespace(SampleStartDelay=1000,SamplePeriod=10,SoundSpeed=1500,SamplesPerBeam=3)
        axes=ArisAxes(row,[-14,-3,7])
        np.testing.assert_allclose(axes.range_axis_m(),[.75375,.76125,.76875])
        np.testing.assert_array_equal(axes.azimuth_axis_deg(),[-14,-3,7])
    def test_se2_alignment_removes_only_rigid_transform(self):
        xy=np.array([[0,0],[1,0],[1,2],[3,4]],float)
        truth=rotate_xy(xy,.7)+[8,-3]
        _,error=score(xy,truth)
        self.assertLess(error.max(),1e-10)
        _,scaled_error=score(2*xy,truth)
        self.assertGreater(scaled_error.max(),.5)
        reflected=xy.copy();reflected[:,0]*=-1
        _,reflection_error=score(reflected,truth)
        self.assertGreater(reflection_error.max(),.1)

if __name__=='__main__':unittest.main(verbosity=2)
