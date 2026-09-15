import unittest,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from evaluate import *
class Checks(unittest.TestCase):
 def test_fast_features_match_original_including_ties(self):
  from types import SimpleNamespace
  rng=np.random.default_rng(120)
  for image in [rng.integers(0,8,(32,90)),np.zeros((32,90)),rng.uniform(0,255,(32,90))]:
   axes=ArisAxes(SimpleNamespace(SampleStartDelay=1000,SamplePeriod=10,SoundSpeed=1500,SamplesPerBeam=90),np.linspace(-15,15,32))
   expected,_=slam.extract_features(image,axes,relative_threshold=.025,max_features=100,exclusion_bins=(2,4))
   np.testing.assert_array_equal(features_fast(image,axes),expected)
 def test_known_rigid_motion_recovery(self):
  rng=np.random.default_rng(17);current=rng.uniform([-1,1],[1,4],(100,2))
  motion=np.array([.08,-.03,.025]);target=slam.transform_points(current,motion)
  estimate,error,n=slam.icp_relative(target,current,max_correspondence_m=.6)
  np.testing.assert_allclose(estimate,motion,atol=1e-8);self.assertLess(error,1e-8)
 def test_accumulation_matches_cpp(self):
  graph=sonar.PlanarSlam(initial=(0,0,0));p=np.zeros(3)
  for m in [[.2,.7,.1],[-.1,.4,-.2],[.3,.1,.3]]:
   graph.add_odometry(m,sigma_translation=.1,sigma_yaw=.1)
   p=np.r_[slam.transform_points(np.array(m)[None,:2],p)[0],p[2]+m[2]]
  np.testing.assert_allclose(p,graph.poses()[-1],atol=1e-12)
 def test_reference_metadata_does_not_enter_input_loader(self):
  import tempfile
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);rec=root/'rec';(rec/'aris_raw').mkdir(parents=True);cache=root/'cache';cache.mkdir()
   row=dict(FrameIndex=0,FrameTime=1000000,SamplesPerBeam=30,SampleStartDelay=1000,SamplePeriod=10,SoundSpeed=1500,SonarPan=0,SonarTilt=-42,SonarRoll=0)
   pd.DataFrame([row,{**row,'FrameIndex':1,'FrameTime':1100000}]).to_csv(rec/'aris_frame_meta.csv',index=False)
   for i in range(2):Image.fromarray(np.arange(30*16,dtype=np.uint8).reshape(30,16)).save(rec/'aris_raw'/f'{i:04d}.pgm')
   angles={16:np.linspace(-15,15,16).tolist()};a=load_inputs(rec,angles,cache)
   meta=pd.read_csv(rec/'aris_frame_meta.csv');meta[['SonarPan','SonarTilt','SonarRoll']]=99999;meta.to_csv(rec/'aris_frame_meta.csv',index=False)
   (rec/'gantry.csv').write_text('not valid ground truth')
   b=load_inputs(rec,angles,cache)
   self.assertEqual(a['cache_key'],b['cache_key'])
   for x,y in zip(a['points'],b['points']):np.testing.assert_array_equal(x,y)
if __name__=='__main__':unittest.main(verbosity=2)
