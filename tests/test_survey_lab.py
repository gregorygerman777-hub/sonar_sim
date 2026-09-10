"""Measurement provenance and native control regression checks for the survey lab."""
import os
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
sys.path[:0] = [str(Path(__file__).resolve().parents[1]/'python'), str(Path(__file__).resolve().parents[1])]
from survey_lab import Survey, App, TARGETS, rotation, boat_geometry
import pygame
import reconstruction as rc
import scene


class SurveyChecks(unittest.TestCase):
    def setUp(self): self.lab = Survey()

    def test_live_measurements_replay_volume_exactly(self):
        lab=self.lab
        for _ in range(3): lab.move(dx=.2,dy=.1)
        images=[r[1] for r in lab.records];poses=[(r[2],r[3]) for r in lab.records]
        expected=rc.reconstruct(images,poses,lab.sim,lab.points)
        np.testing.assert_array_equal(lab.kept,expected)
        for raw,image,pos,axes in lab.records:
            np.testing.assert_allclose(raw,lab.sim.render(lab.objects,pos,axes),rtol=1e-12,atol=1e-20)

    def test_gain_camera_pause_preserve_measurement(self):
        lab=self.lab;lab.running=False;raw=lab.raw.copy();count=lab.ping_number
        lab.set_parameter('gain',12);lab.set_parameter('speed',.4);lab.step(.1)
        app=App(lab);app.action('top');app.action('water');app.draw()
        np.testing.assert_array_equal(raw,lab.raw);self.assertEqual(count,lab.ping_number)
        pygame.quit()

    def test_motion_updates_pose_and_measurement_together(self):
        lab=self.lab;before=lab.raw.copy();start=lab.position.copy()
        for _ in range(4):lab.step(.1)
        self.assertGreater(np.linalg.norm(lab.position-start),0)
        self.assertGreater(np.max(np.abs(lab.raw-before)),0)
        np.testing.assert_array_equal(lab.position,lab.ping_position)
        np.testing.assert_array_equal(lab.axes,lab.ping_axes)

    def test_target_meshes_and_reset(self):
        lab=self.lab
        for target in TARGETS:
            lab.set_parameter('target',target)
            self.assertTrue(np.isfinite(lab.raw).all());self.assertGreater(lab.raw.max(),0)
            self.assertEqual(len(lab.records),1)
            v,_=lab.target_geometry
            self.assertGreaterEqual(v[:,2].min(),-3)

    def test_noise_is_same_image_used_by_carving(self):
        lab=self.lab;raw=lab.raw.copy();lab.set_parameter('noise',True)
        np.testing.assert_array_equal(raw,lab.raw)
        self.assertFalse(np.array_equal(lab.raw,lab.image))
        np.testing.assert_array_equal(lab.records[-1][1],lab.image)
        expected=rc.reconstruct([lab.image],[(lab.position,lab.axes)],lab.sim,lab.points)
        np.testing.assert_array_equal(expected,lab.kept)

    def test_config_changes_reset_and_unobserved_survive(self):
        lab=self.lab;lab.move(dx=.2);lab.set_parameter('range',8.)
        self.assertEqual(len(lab.records),1)
        self.assertTrue(lab.kept[~lab.observed].all())

    def test_beam_geometry_uses_acoustic_pose(self):
        lab=self.lab;lab.set_parameter('heading',23);lab.set_parameter('tilt',31)
        for theta in (-lab.fov/2,0,lab.fov/2):
            for phi in (-lab.beam/2,lab.beam/2):
                world=lab.position+lab.axes@scene.spherical_to_world(lab.range,theta,phi)
                local=(world-lab.position)@lab.axes
                self.assertAlmostEqual(np.linalg.norm(local),lab.range)
                self.assertAlmostEqual(np.degrees(np.arctan2(local[0],local[1])),theta)
                self.assertAlmostEqual(np.degrees(np.arctan2(local[2],np.hypot(*local[:2]))),phi)
        np.testing.assert_allclose(lab.axes.T@lab.axes,np.eye(3),atol=1e-14)

    def test_export_round_trip(self):
        lab=self.lab;lab.move(dx=.1)
        with tempfile.TemporaryDirectory() as d:
            lab.export(Path(d));data=np.load(Path(d)/'survey.npz')
            np.testing.assert_array_equal(data['images'][-1],lab.image)
            np.testing.assert_array_equal(data['positions'][-1],lab.position)
            np.testing.assert_array_equal(data['kept'],lab.kept)

    def test_cap_does_not_desynchronize_pose(self):
        lab=self.lab;lab.records=lab.records*500;position=lab.position.copy();heading=lab.heading
        lab.move(dx=1);lab.set_parameter('heading',42)
        np.testing.assert_array_equal(lab.position,position);self.assertEqual(lab.heading,heading)

    def test_capture_record_and_orbit(self):
        lab=self.lab;app=App(lab);app.draw();before=lab.raw.copy()
        with tempfile.TemporaryDirectory() as d:
            app.output=Path(d)
            app.action('capture');self.assertTrue(list(Path(d).glob('capture_*.png')))
            self.assertTrue((Path(d)/'survey.npz').exists())
            app.action('record');app.draw();app.action('record')
            self.assertTrue((Path(d)/'frames/0000.png').exists())
        app.handle(pygame.event.Event(pygame.MOUSEWHEEL,y=1))
        self.assertLess(app.camera.distance,12.)
        old=app.camera.az
        app.handle(pygame.event.Event(pygame.MOUSEBUTTONDOWN,button=3,pos=(400,300)))
        app.handle(pygame.event.Event(pygame.MOUSEMOTION,pos=(420,300),rel=(20,0)))
        self.assertNotEqual(old,app.camera.az)
        np.testing.assert_array_equal(before,lab.raw)
        pygame.quit()

    def test_buttons_sliders_keyboard_and_layout(self):
        lab=self.lab;app=App(lab,(1600,1000));app.draw()
        for rect,action in list(app.buttons):
            self.assertGreaterEqual(rect.h,44)
            self.assertTrue(pygame.Rect(0,0,1600,1000).contains(rect))
            if action not in ('capture','export','record'):
                app.handle(pygame.event.Event(pygame.MOUSEBUTTONDOWN,button=1,pos=rect.center))
        app.draw()
        for rect,name,low,high in app.sliders:
            app.handle(pygame.event.Event(pygame.MOUSEBUTTONDOWN,button=1,pos=(rect.centerx,rect.centery)))
            app.handle(pygame.event.Event(pygame.MOUSEBUTTONUP,button=1,pos=rect.center))
            self.assertAlmostEqual(getattr(lab,name),(low+high)/2,places=2)
        lab.running=False
        app.handle(pygame.event.Event(pygame.KEYDOWN,key=pygame.K_d))
        np.testing.assert_array_equal(lab.position,lab.ping_position)
        for size in ((1000,625),(1440,900),(1600,1000)):
            app.handle(pygame.event.Event(pygame.VIDEORESIZE,w=size[0],h=size[1]));app.draw()
            self.assertEqual(app.screen.get_size(),size)
        pygame.quit()


if __name__=='__main__':unittest.main(verbosity=2)
