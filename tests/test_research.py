import sys,unittest,tempfile
from pathlib import Path
sys.path[:0]=['.','python']
import numpy as np
import sonar,scene as sc,speckle,point_recovery as pr

class ResearchChecks(unittest.TestCase):
    def test_triangle_and_mesh(self):
        self.assertAlmostEqual(sonar.ray_triangle((-1,4,-1),(1,4,-1),(0,4,1),(0,0,0),(0,1,0)),4)
        self.assertIsNone(sonar.ray_triangle((-1,4,-1),(1,4,-1),(0,4,1),(3,0,0),(0,1,0)))
        sim=sonar.SonarSimulator(num_azimuth_bins=1,num_range_bins=100,max_range_m=10,num_elevation_subrays=1)
        im=sim.render([sonar.make_mesh('assets/box.obj',translation=(0,4,0))])
        self.assertEqual(int(np.argmax(im)),35)
        self.assertAlmostEqual(im[0,35],.8/3.5**4*10**(-sonar.two_way_loss_db(sonar.thorp_alpha(1.8e6),3.5)/10))
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'bad.obj';p.write_text('v 0 0 0\nf 1 2 3\n')
            with self.assertRaises(RuntimeError):sim.render([sonar.make_mesh(p)])

    def test_projection_and_carving(self):
        sim=sonar.SonarSimulator(num_azimuth_bins=10,num_range_bins=100,max_range_m=10,vertical_beamwidth_deg=30)
        p=sonar.project_bins([(0,4,0),(0,10,0),(0,-4,0)],sim)
        self.assertEqual(p[0][:3],(True,5,40));self.assertFalse(p[1][0]);self.assertFalse(p[2][0])
        h=np.zeros(sim.shape,bool);s=h.copy();h[5,40]=True
        kept=sonar.carve([(0,4,0),(0,2,0),(0,-4,0)],sim,h,s,bearing_tolerance_bins=0,range_tolerance_bins=0)
        np.testing.assert_array_equal(kept,[True,False,True])

    def test_speckle(self):
        for correlation in ((0,0),(2,3)):
            f=speckle.factors((1024,1024),42,correlation)
            self.assertLess(abs(f.mean()-1),.03);self.assertLess(abs(f.std()/f.mean()-1),.04)
            self.assertLess(abs(np.mean(f<.1)-(1-np.exp(-.1))),.01)
            np.testing.assert_array_equal(f,speckle.factors(f.shape,42,correlation))

    def test_ambiguity_and_threads(self):
        cfg=dict(num_azimuth_bins=65,num_range_bins=200,num_elevation_subrays=256,vertical_beamwidth_deg=30)
        sim=sonar.SonarSimulator(**cfg)
        pair=[sim.render([sonar.make_sphere(sc.spherical_to_world(4,3,p),.2)]) for p in (-5,5)]
        self.assertLess(np.max(abs(pair[0]-pair[1]))/pair[0].max(),1e-11)
        objects=[sonar.make_sphere((0,4,-.5),.5)]
        frames=[sonar.SonarSimulator(**cfg,num_threads=n,multipath_enabled=True).render(objects,position=(0,0,-1),axes=sc.tilted_axes(0,60)) for n in (1,2,8)]
        for f in frames[1:]:np.testing.assert_allclose(f,frames[0],rtol=1e-12,atol=1e-20)

    def test_components_and_normalization(self):
        objects=[sonar.make_plane((0,4,0),(0,-1,0),.8)]
        cfg=dict(num_azimuth_bins=21,num_range_bins=100,num_elevation_subrays=256,vertical_beamwidth_deg=30,multipath_enabled=True)
        frames=[]
        for key in ('direct_enabled','ghost_enabled','mirror_enabled'):
            flags=dict(direct_enabled=False,ghost_enabled=False,mirror_enabled=False);flags[key]=True
            frames.append(sonar.SonarSimulator(**cfg,**flags).render(objects,position=(0,0,-.2)))
        combined=sonar.SonarSimulator(**cfg).render(objects,position=(0,0,-.2))
        np.testing.assert_allclose(sum(frames),combined,rtol=1e-12,atol=1e-20)
        sums=[sonar.SonarSimulator(**dict(cfg,num_elevation_subrays=n,multipath_enabled=False)).render(objects).sum() for n in (512,1024)]
        self.assertLess(abs(sums[0]/sums[1]-1),1e-4)
        self.assertEqual(sonar.beam_response(0,mode='hann'),1.)

    def test_recovery_degeneracy(self):
        poses=[(np.array([x,0,0]),np.eye(3)) for x in (-.3,.3)]
        np.testing.assert_allclose(pr.observe([.3,4,.5],poses),pr.observe([.3,4,-.5],poses))
        self.assertEqual(np.linalg.matrix_rank(pr.jacobian([.3,4,0],poses)),2)

if __name__=='__main__':unittest.main(verbosity=2)
