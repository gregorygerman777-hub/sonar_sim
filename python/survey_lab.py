"""Boat-mounted survey: one acoustic scene, ping state and live feasible volume.

Native Pygame perspective renderer; no external assets or new dependencies.
Presentation hull and water are not acoustic scatterers. Transducer is below keel.
"""
from pathlib import Path
from datetime import datetime
import argparse
import json
import sys
import time
import numpy as np
import pygame

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'python')]
import sonar
import scene
import mesh_assets as meshes
import reconstruction as rc
import speckle

# Instrument tokens: blue-biased neutrals, cyan accent, amber ground-truth semantic.
BG = (10, 19, 26)
PANEL = (16, 30, 39)
EDGE = (48, 70, 80)
TEXT = (228, 237, 239)
MUTED = (155, 178, 189)
CYAN = (105, 226, 212)
TRUTH = (235, 179, 103)
SPACING = (4, 8, 16, 24, 32, 48, 64)
W, H = 1600, 1000
TARGETS = ('sphere', 'cylinder', 'pipe', 'wreck')


def rotation(yaw, tilt=0):
    a = np.radians(yaw)
    return np.array([[np.cos(a), -np.sin(a), 0],
                     [np.sin(a), np.cos(a), 0], [0, 0, 1]]) @ scene.tilted_axes(tilt)


def combine(parts):
    vs, fs = [], []
    for v, f in parts:
        fs.extend(f + len(vs)); vs.extend(v)
    return np.array(vs, float), np.array(fs, int)


def cylinder(n=24):
    a = np.arange(n) * 2 * np.pi / n
    v = np.array([[.4*np.cos(t), .4*np.sin(t), z] for z in (-.55, .55) for t in a]
                 + [[0, 0, -.55], [0, 0, .55]])
    f = []
    for i in range(n):
        j = (i+1) % n
        f.extend([(i, j, n+j), (i, n+j, n+i), (2*n, j, i), (2*n+1, n+i, n+j)])
    return v, np.array(f)


def target_mesh(name):
    if name == 'sphere': return meshes.radial_target(rings=12, segments=24)
    if name == 'cylinder': return cylinder()
    if name == 'pipe':
        v, f = meshes.pipe(24)
        return v * [1.7, 1.6, 1.6], f
    # Closed components representing a broken hull, deck and transverse ribs.
    parts = [meshes.box((-.9, -.6, -.32), (.9, .6, -.22)),
             meshes.box((-.9, -.6, -.22), (-.78, .6, .18)),
             meshes.box((.78, -.6, -.22), (.9, .15, .12)),
             meshes.box((-.4, -.1, -.22), (.1, .35, .48))]
    for y in (-.48, .1, .48):
        parts.append(meshes.box((-.78, y, -.22), (.78, y+.06, -.05)))
    return combine(parts)


def write_obj(path, v, f):
    path.write_text('# Procedural survey geometry; metres, X right Y forward Z up\n' +
                    ''.join('v %.9g %.9g %.9g\n' % tuple(p) for p in v) +
                    ''.join('f %d %d %d\n' % tuple(p+1) for p in f))


class Survey:
    """Measurement state independent of camera, display gain and window size."""
    def __init__(self):
        self.asset_dir = ROOT / 'output' / 'survey_assets'
        self.asset_dir.mkdir(parents=True, exist_ok=True)
        self.target = 'sphere'
        self.position = np.array([-.8, -.8, -.65])
        self.heading, self.tilt = -8., 20.
        self.range, self.frequency, self.beam, self.fov = 10., 600., 30., 60.
        self.speed, self.gain = .18, 0.
        self.noise = False; self.multipath = False; self.running = True
        self.t = 0.; self.last_ping = -1.; self.ping_number = 0
        self.records = []; self.render_ms = 0.; self.status = 'Survey in progress'
        self.centre = np.array([0., 4.5, -2.4])
        self.build_scene(); self.clear_volume(); self.ping()

    def build_scene(self):
        v, f = target_mesh(self.target)
        path = self.asset_dir / (self.target+'.obj'); write_obj(path, v, f)
        obj = (sonar.make_sphere(self.centre, .5, .8) if self.target == 'sphere'
               else sonar.make_mesh(str(path), translation=self.centre, reflectivity=.8))
        self.objects = [sonar.make_plane((0, 0, -3), (0, 0, 1), .045), obj]
        self.target_geometry = (v+self.centre, f)
        self.draw_meshes = [(v+self.centre, f, TRUTH)]
        # These rocks use identical mesh triangles in the visual and acoustic scenes.
        rv, rf = meshes.radial_target(True, rings=7, segments=12)
        for i, (p, scale) in enumerate([((-2.2, 5.5, -2.7), (.9, 1.2, .6)),
                                       ((1.9, 7.2, -2.65), (1.2, .9, .7)),
                                       ((-2.8, 8.3, -2.8), (.7, .9, .4))]):
            world = rv*np.array(scale)+p
            path = self.asset_dir/f'rock_{i}.obj'; write_obj(path, world, rf)
            self.objects.append(sonar.make_mesh(str(path), reflectivity=.12))
            self.draw_meshes.append((world, rf, (113, 132, 137)))

    def clear_volume(self):
        # Fixed operator ROI, independent of selected target mesh or occupancy.
        self.voxel = .12
        axes = [np.arange(-1.44, 1.45, self.voxel), np.arange(3., 6.01, self.voxel),
                np.arange(-2.94, -1.13, self.voxel)]
        self.points = np.stack(np.meshgrid(*axes, indexing='ij'), axis=-1).reshape(-1, 3)
        self.kept = np.ones(len(self.points), bool)
        self.observed = np.zeros(len(self.points), bool)
        self.records = []

    def ping(self):
        if len(self.records) >= 500:
            self.running = False; self.status = '500-ping limit. Export or reset survey.'
            return
        start = time.perf_counter()
        self.axes = rotation(self.heading, self.tilt)
        self.config = dict(frequency_hz=self.frequency*1000, num_azimuth_bins=97,
                           num_range_bins=256, max_range_m=self.range,
                           horizontal_fov_deg=self.fov, vertical_beamwidth_deg=self.beam,
                           num_elevation_subrays=512, beam_mode='top_hat', num_threads=4,
                           multipath_enabled=self.multipath, surface_z=0)
        self.sim = sonar.SonarSimulator(**self.config)
        self.raw = self.sim.render(self.objects, self.position, self.axes)
        self.image = self.raw * speckle.factors(self.raw.shape, self.ping_number+19) if self.noise else self.raw.copy()
        self.ping_position = self.position.copy(); self.ping_axes = self.axes.copy()
        hi, shadow = rc.segment(self.image, .03)
        self.kept = sonar.carve(self.points, self.sim, hi, shadow,
                                self.ping_position, self.ping_axes, self.kept, 1, 1)
        local = (self.points-self.ping_position) @ self.ping_axes
        theta = np.degrees(np.arctan2(local[:, 0], local[:, 1]))
        phi = np.degrees(np.arctan2(local[:, 2], np.hypot(local[:, 0], local[:, 1])))
        self.observed |= ((np.abs(theta) < self.fov/2) & (np.abs(phi) <= self.beam/2)
                          & (np.linalg.norm(local, axis=1) < self.range))
        self.records.append((self.raw.copy(), self.image.copy(), self.ping_position.copy(), self.ping_axes.copy()))
        self.ping_number += 1; self.last_ping = self.t
        self.render_ms = (time.perf_counter()-start)*1000
        if len(self.records) >= 500:
            self.running = False; self.status = '500-ping limit. Export or reset survey.'

    def set_parameter(self, name, value):
        if getattr(self, name) == value: return
        if len(self.records) >= 500 and name in ('heading', 'tilt'):
            self.status = '500-ping limit. Export or reset before changing pose.'
            return
        setattr(self, name, value)
        if name in ('gain', 'speed'): return
        if name == 'target': self.build_scene()
        if name not in ('heading', 'tilt'):
            self.clear_volume()  # Do not silently mix different acquisition settings.
        self.ping()

    def move(self, dx=0, dy=0):
        if len(self.records) >= 500: return
        self.position[:2] += [dx, dy]; self.ping()

    def reset(self):
        self.position[:] = [-.8, -.8, -.65]; self.t = 0.; self.heading = -8.
        self.clear_volume(); self.status = 'Survey reset'; self.ping()

    def step(self, dt):
        # Commit movement and sonar together at acquisition time (4 Hz nominal).
        if not self.running: return
        dt = min(dt, .1); self.t += dt
        if self.t-self.last_ping >= .25:
            travel = (self.t-self.last_ping)*self.speed
            self.position += rotation(self.heading)[:, 1]*travel
            if np.linalg.norm(self.position[:2]) > 12:
                self.running = False; self.status = 'Survey boundary reached. Reset to return.'
            self.ping()

    def export(self, directory):
        directory.mkdir(parents=True, exist_ok=True)
        raw, images, positions, axes = zip(*self.records)
        np.savez_compressed(directory/'survey.npz', raw=raw, images=images, positions=positions,
                            axes=axes, points=self.points, kept=self.kept, observed=self.observed)
        metadata = dict(config=self.config, target=self.target, pings=len(self.records),
                        voxel_m=self.voxel, segmentation_fraction=.03, tolerance_bins=1,
                        noise=self.noise, display_gain_db=self.gain, simulation_seconds=self.t,
                        reconstruction='Live displayed measurements, fixed operator ROI, conservative farther-range mask. Unobserved voxels retained.',
                        approximations='Ideal top-hat beam, diffuse angular mean, optional planar multipath. No hardware calibration or ghost removal. Boat/water visual only; transducer below keel.')
        (directory/'survey.json').write_text(json.dumps(metadata, indent=2))


class Camera:
    def __init__(self): self.az = -.72; self.el = .48; self.distance = 12.
    def project(self, points, rect, focus):
        eye = np.array(focus)+self.distance*np.array([np.sin(self.az)*np.cos(self.el),
                                                     -np.cos(self.az)*np.cos(self.el), np.sin(self.el)])
        forward = (focus-eye)/self.distance
        right = np.cross(forward, [0, 0, 1]); right /= np.linalg.norm(right)
        up = np.cross(right, forward)
        d = np.asarray(points)-eye; depth = d@forward
        scale = min(rect.w, rect.h)*1.25
        xy = np.stack([rect.centerx+scale*(d@right)/np.maximum(depth, .1),
                       rect.centery-scale*(d@up)/np.maximum(depth, .1)], axis=1)
        return np.clip(xy, -20000, 20000).astype(int), depth


def boat_geometry(position, heading):
    # Bow points along +Y. All dimensions are in metres.
    ring = np.array([[-.55, -.95], [.55, -.95], [.62, .5], [.36, 1.15], [0, 1.48], [-.36, 1.15], [-.62, .5]])
    n = len(ring)
    v = np.vstack([np.column_stack([ring*.72, np.full(n, -.28)]),
                   np.column_stack([ring, np.full(n, .22)]), [[0, 0, -.28], [0, 0, .22]]])
    f = []
    for i in range(n):
        j = (i+1)%n
        f.extend([(i, j, n+j), (i, n+j, n+i), (2*n, j, i), (2*n+1, n+i, n+j)])
    parts = [(v, np.array(f), (190, 202, 197)),
             (*meshes.box((-.37, -.5, .22), (.37, .32, .8)), (210, 215, 203)),
             (*meshes.box((-.4, -.54, .8), (.4, .36, .86)), (102, 131, 141)),
             (*meshes.box((-.3, .322, .45), (.3, .33, .71)), (37, 78, 97)),
             (*meshes.box((-.375, -.37, .45), (-.369, .15, .71)), (37, 78, 97)),
             (*meshes.box((.369, -.37, .45), (.375, .15, .71)), (37, 78, 97)),
             (*meshes.box((-.025, -.2, .85), (.025, -.15, 1.6)), (193, 207, 210)),
             (*meshes.box((-.25, -.2, 1.45), (.25, -.14, 1.5)), (193, 207, 210)),
             (*meshes.box((-.035, -.035, -.59), (.035, .035, -.2)), (73, 101, 117))]
    mount = np.array([position[0], position[1], 0.])
    return [(v@rotation(heading).T+mount, f, color) for v, f, color in parts]


class App:
    def __init__(self, lab, size=(1440, 900)):
        pygame.init(); self.screen = pygame.display.set_mode(size, pygame.RESIZABLE)
        pygame.display.set_caption('FSS Survey Laboratory | boat-mounted sonar')
        self.canvas = pygame.Surface((W, H)); self.lab = lab; self.camera = Camera()
        self.title = pygame.font.SysFont('Georgia', 28)
        self.body = pygame.font.SysFont('Arial', 16)
        self.mono = pygame.font.SysFont('Menlo', 12)
        self.scene_rect = pygame.Rect(24, 136, 928, 480)
        self.sonar_rect = pygame.Rect(976, 136, 600, 480)
        self.recon_rect = pygame.Rect(24, 656, 576, 232)
        self.clear_water = True; self.full = False; self.drag = False; self.held = None
        self.buttons = []; self.sliders = []; self.recording = False; self.record_frames = 0
        self.done = False; self.notice = ''; self.last_record = 0.
        self.output = ROOT/'results'/('survey_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        self.output.mkdir(parents=True)

    def text(self, text, x, y, color=TEXT, font=None):
        self.canvas.blit((font or self.body).render(str(text), True, color), (x, y))

    def button(self, label, rect, action, active=False):
        rect = pygame.Rect(rect); self.buttons.append((rect, action))
        pygame.draw.rect(self.canvas, (27, 62, 66) if active else PANEL, rect, border_radius=3)
        pygame.draw.rect(self.canvas, CYAN if active else EDGE, rect, 1, border_radius=3)
        self.text(label, rect.x+8, rect.y+12, CYAN if active else TEXT)

    def slider(self, label, name, low, high, x, y, width=208, unit=''):
        value = getattr(self.lab, name)
        self.text(f'{label}  {value:.1f}{unit}', x, y, MUTED, self.mono)
        rect = pygame.Rect(x, y+20, width, 24)
        self.sliders.append((rect, name, low, high))
        pygame.draw.line(self.canvas, EDGE, (x, y+32), (x+width, y+32), 3)
        px = x+width*(value-low)/(high-low)
        pygame.draw.circle(self.canvas, CYAN, (int(px), y+32), 5)

    def action(self, name):
        lab = self.lab
        if name == 'pause': lab.running = not lab.running
        elif name == 'reset': lab.reset()
        elif name == 'ping': lab.ping()
        elif name == 'target': lab.set_parameter('target', TARGETS[(TARGETS.index(lab.target)+1)%len(TARGETS)])
        elif name in ('noise', 'multipath'): lab.set_parameter(name, not getattr(lab, name))
        elif name == 'water': self.clear_water = not self.clear_water
        elif name == 'camera':
            self.camera.az, self.camera.el, self.camera.distance = -.72, .48, 12.
        elif name == 'top': self.camera.az, self.camera.el, self.camera.distance = 0., 1.48, 14.
        elif name == 'capture':
            pygame.image.save(self.canvas, self.output/f'capture_{time.time_ns()}.png')
            lab.export(self.output); self.notice = 'Saved image + measurements to '+self.output.name
        elif name == 'export': lab.export(self.output); self.notice = 'Saved survey.npz + survey.json to '+self.output.name
        elif name == 'record':
            if self.record_frames >= 300:
                self.notice = 'Recording cap reached. Start a new session to record again.'
                return
            self.recording = not self.recording
            self.notice = 'Recording PNG sequence (300-frame cap)' if self.recording else 'Recording stopped'

    def handle(self, event):
        if event.type == pygame.QUIT: self.done = True
        elif event.type == pygame.VIDEORESIZE and not self.full:
            self.screen = pygame.display.set_mode((max(1000, event.w), max(625, event.h)), pygame.RESIZABLE)
        elif event.type == pygame.KEYDOWN:
            actions = {pygame.K_SPACE:'pause', pygame.K_r:'reset', pygame.K_RETURN:'ping', pygame.K_m:'target',
                       pygame.K_n:'noise', pygame.K_g:'multipath', pygame.K_c:'water', pygame.K_p:'capture',
                       pygame.K_e:'export', pygame.K_v:'record', pygame.K_1:'camera', pygame.K_2:'top'}
            if event.key in actions: self.action(actions[event.key])
            elif event.key in (pygame.K_ESCAPE, pygame.K_q): self.done = True
            elif event.key == pygame.K_f:
                self.full = not self.full
                self.screen = pygame.display.set_mode((0, 0) if self.full else (1440, 900), pygame.FULLSCREEN if self.full else pygame.RESIZABLE)
            elif event.key in (pygame.K_a, pygame.K_d): self.lab.move(dx=.2 if event.key == pygame.K_d else -.2)
            elif event.key in (pygame.K_w, pygame.K_s): self.lab.move(dy=.2 if event.key == pygame.K_w else -.2)
        elif event.type == pygame.MOUSEWHEEL:
            self.camera.distance = np.clip(self.camera.distance-event.y*.6, 5., 22.)
        elif event.type in (pygame.MOUSEBUTTONDOWN, pygame.MOUSEMOTION, pygame.MOUSEBUTTONUP):
            pos = (event.pos[0]*W/self.screen.get_width(), event.pos[1]*H/self.screen.get_height())
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 3 and self.scene_rect.collidepoint(pos): self.drag = True
                if event.button == 1:
                    for rect, action in self.buttons:
                        if rect.collidepoint(pos): self.action(action); break
                    for spec in self.sliders:
                        if spec[0].collidepoint(pos): self.held = spec; break
            elif event.type == pygame.MOUSEBUTTONUP:
                self.drag = False; self.held = None
            elif self.drag:
                self.camera.az += event.rel[0]*.005
                self.camera.el = np.clip(self.camera.el+event.rel[1]*.004, .12, 1.48)
            if self.held:
                rect, name, low, high = self.held
                value = float(np.clip((pos[0]-rect.x)/rect.w, 0, 1)*(high-low)+low)
                self.lab.set_parameter(name, round(value, 2))

    def draw_world(self):
        lab = self.lab; rect = self.scene_rect
        self.canvas.set_clip(rect); pygame.draw.rect(self.canvas, (13, 38, 48), rect)
        focus = np.array([0., 3., -1.3])
        project = lambda p: self.camera.project(p, rect, focus)
        def line(a, b, color=EDGE, width=1):
            xy, depth = project([a, b])
            if np.all(depth > .1): pygame.draw.line(self.canvas, color, xy[0], xy[1], width)
        polygons = []
        def add(v, f, color, alpha=255):
            xy, depth = project(v)
            for face in f:
                if np.any(depth[face] <= .1): continue
                normal = np.cross(v[face[1]]-v[face[0]], v[face[2]]-v[face[0]])
                normal /= max(np.linalg.norm(normal), 1e-8)
                light = .4+.6*abs(normal@np.array([-.35, -.4, .84]))
                polygons.append((depth[face].mean(), xy[face], tuple((np.array(color)*light).astype(int)), alpha))
        # A metre grid rendered as seabed tiles, matching the acoustic plane z=-3.
        for x in range(-5, 5):
            for y in range(-3, 12):
                v = np.array([[x,y,-3],[x+1,y,-3],[x+1,y+1,-3],[x,y+1,-3]], float)
                add(v, [[0,1,2,3]], (51+(x+y)%2*5, 69+(x+y)%2*5, 70+(x+y)%2*5))
        for v, f, color in lab.draw_meshes+boat_geometry(lab.ping_position, lab.heading): add(v, f, color)
        v, f = meshes.box((-.15, -.1, -.07), (.15, .1, .07))
        add(v@lab.ping_axes.T+lab.ping_position, f, CYAN)
        # Exact spherical support boundary: same FOV, beam, range and pose as ping.
        p, axes = lab.ping_position, lab.ping_axes
        bands = []
        for phi in (-lab.beam/2, lab.beam/2):
            bands.append(np.array([p+axes@scene.spherical_to_world(lab.range, t, phi)
                                  for t in np.linspace(-lab.fov/2, lab.fov/2, 25)]))
        for band in bands:
            for a, b in zip(band, band[1:]): add(np.array([p, a, b]), [[0,1,2]], CYAN, 12)
        for i in (0, -1): add(np.array([p,bands[0][i],bands[1][i]]), [[0,1,2]], CYAN, 18)
        if not self.clear_water:
            water = np.array([[-5,-3,0],[5,-3,0],[5,12,0],[-5,12,0]], float)
            add(water, [[0,1,2,3]], (55, 119, 140), 105)
        for _, poly, color, alpha in sorted(polygons, key=lambda x:-x[0]):
            if alpha == 255: pygame.draw.polygon(self.canvas, color, poly)
            else:
                # Composite only a polygon's visible bounds, not 50 full windows.
                lo=poly.min(axis=0); hi=poly.max(axis=0)
                bounds=pygame.Rect(*lo,*(hi-lo+1)).clip(rect)
                if bounds.w and bounds.h:
                    overlay=pygame.Surface(bounds.size,pygame.SRCALPHA)
                    pygame.draw.polygon(overlay,(*color,alpha),poly-np.array(bounds.topleft))
                    self.canvas.blit(overlay,bounds.topleft)
        for band in bands:
            for a, b in zip(band, band[1:]): line(a,b,(60,130,132))
            for end in (band[0],band[-1]): line(p,end,(60,130,132))
        # Sparse water contours and wake are explicitly presentation geometry.
        for x in (-4, -2, 0, 2, 4):
            pts = [[x,y,.025*np.sin(y*2+lab.t)] for y in np.linspace(-2,11,40)]
            for a,b in zip(pts,pts[1:]): line(a,b,(39,91,107))
        track = [record[2]+[0,0,.65] for record in lab.records]
        for a,b in zip(track,track[1:]): line(a,b,(171,194,201),2)
        route_origin = lab.ping_position+[0,0,.65]
        route_direction = rotation(lab.heading)[:,1]
        for distance in np.arange(0,7,.6):
            line(route_origin+route_direction*distance,
                 route_origin+route_direction*(distance+.25),(137,174,184),1)
        for axis, label, color in [([1,0,0],'X',(202,157,137)),([0,1,0],'Y',CYAN),([0,0,1],'Z',TEXT)]:
            origin=np.array([-3.,1.,-2.98]); end=origin+axis
            line(origin,end,color,2); xy,depth=project([end])
            if depth[0]>.1: self.text(label,*xy[0],color,self.mono)
        line([-2,8,-2.98],[0,8,-2.98],TEXT,3)
        xy,depth=project([[-1,8,-2.9]])
        if depth[0]>.1: self.text('2 m',*xy[0],TEXT,self.mono)
        xy,depth=project([p])
        if depth[0]>.1: self.text('TRANSDUCER',xy[0,0]+12,xy[0,1],CYAN,self.mono)
        self.canvas.set_clip(None)
        self.text('Right-drag: orbit   Wheel: zoom   Grid: 1 m',rect.x+16,rect.bottom-24,MUTED,self.mono)

    def draw_sonar(self):
        lab = self.lab; rect = self.sonar_rect
        yy,xx=np.mgrid[:rect.h,:rect.w]; dx=xx-rect.w/2; dy=rect.h-40-yy
        radius=rect.h-64; rr=np.hypot(dx,dy); theta=np.arctan2(dx,dy)
        fov=np.radians(lab.fov)
        valid=(dy>=0)&(rr<radius)&(abs(theta)<fov/2)
        az=np.clip(((theta/fov+.5)*97).astype(int),0,96)
        rb=np.clip((rr/radius*256).astype(int),0,255)
        level=np.clip((scene.to_decibels(lab.image)+lab.gain+60)/60,0,1)[az,rb]
        level=np.where(valid,level,0)
        rgb=np.stack([16+level*180,30+level*205,39+level*179],axis=-1).astype(np.uint8)
        self.canvas.blit(pygame.surfarray.make_surface(rgb.transpose(1,0,2)),rect)
        origin=(rect.centerx,rect.bottom-40)
        for fraction in (.25,.5,.75,1.):
            points=[(origin[0]+np.sin(a)*radius*fraction,origin[1]-np.cos(a)*radius*fraction) for a in np.linspace(-fov/2,fov/2,65)]
            pygame.draw.lines(self.canvas,EDGE,False,points)
            self.text(f'{lab.range*fraction:g} m',rect.x+16,origin[1]-radius*fraction,MUTED,self.mono)
        for angle in (-lab.fov/2,0,lab.fov/2):
            a=np.radians(angle); end=(origin[0]+np.sin(a)*radius,origin[1]-np.cos(a)*radius)
            pygame.draw.line(self.canvas,EDGE,origin,end)
            self.text(f'{angle:+.0f} deg',end[0]-24,end[1]-20,MUTED,self.mono)
        pygame.draw.circle(self.canvas,CYAN,origin,5)
        self.text('Range + bearing / per-ping peak = 0 dB',rect.x+16,rect.bottom-24,MUTED,self.mono)
        for i in range(160):
            value=i/159
            pygame.draw.line(self.canvas,(int(16+value*180),int(30+value*205),int(39+value*179)),
                             (rect.right-32,rect.y+40+i),(rect.right-24,rect.y+40+i))
        self.text('0 dB',rect.right-64,rect.y+208,MUTED,self.mono)
        self.text('-60',rect.right-64,rect.y+16,MUTED,self.mono)

    def draw_reconstruction(self):
        lab=self.lab;rect=self.recon_rect;self.canvas.set_clip(rect)
        pygame.draw.rect(self.canvas,PANEL,rect)
        camera=Camera();camera.az=self.camera.az;camera.el=.55;camera.distance=4.8
        xy,depth=camera.project(lab.points[lab.kept & lab.observed],rect,lab.centre)
        for q,z in zip(xy[::2],depth[::2]):
            if z>.1: pygame.draw.circle(self.canvas,(60,158,156),q,2)
        v,f=lab.target_geometry;xy,depth=camera.project(v,rect,lab.centre)
        for tri in f:
            if np.all(depth[tri]>.1): pygame.draw.lines(self.canvas,TRUTH,True,xy[tri],1)
        self.canvas.set_clip(None)
        self.text('Cyan: observed feasible voxels   Amber: true target',rect.x+8,rect.bottom-24,MUTED,self.mono)

    def draw(self):
        lab=self.lab;self.buttons=[];self.sliders=[];self.canvas.fill(BG)
        self.text('FSS / Survey laboratory',24,24,TEXT,self.title)
        self.text('BOAT-MOUNTED FORWARD-SCAN SONAR',24,64,CYAN,self.mono)
        self.text(f'{lab.t:06.1f} s   PING {lab.ping_number:04d}   {lab.render_ms:.0f} ms / acquisition',664,32,MUTED,self.mono)
        self.button('Pause' if lab.running else 'Run',(1160,24,96,44),'pause',lab.running)
        self.button('Reset',(1264,24,88,44),'reset')
        self.button('Capture',(1360,24,96,44),'capture')
        self.button('Export',(1464,24,112,44),'export')
        self.text('01 / PHYSICAL SCENE',24,108,MUTED,self.mono)
        self.text('02 / LIVE SONAR MEASUREMENT',976,108,MUTED,self.mono)
        self.draw_world();self.draw_sonar()
        self.text('03 / LIVE SURVEY FEASIBLE VOLUME',24,632,MUTED,self.mono)
        self.draw_reconstruction()
        self.text('ACQUISITION / range, frequency, beam and FOV reset volume; gain is display only',624,640,MUTED,self.mono)
        self.slider('Range','range',5,16,624,672,unit=' m')
        self.slider('Frequency','frequency',100,1200,864,672,unit=' kHz')
        self.slider('Vertical beam','beam',8,50,1104,672,unit=' deg')
        self.slider('Horizontal FOV','fov',20,90,1344,672,unit=' deg')
        self.slider('Heading','heading',-60,60,624,736,unit=' deg')
        self.slider('Sonar tilt','tilt',0,45,864,736,unit=' deg')
        self.slider('Boat speed','speed',0,.5,1104,736,unit=' m/s')
        self.slider('Display gain','gain',-20,20,1344,736,unit=' dB')
        self.button('Target: '+lab.target,(624,800,184,44),'target')
        self.button('Speckle',(816,800,112,44),'noise',lab.noise)
        self.button('Multipath',(936,800,120,44),'multipath',lab.multipath)
        self.button('Clear water',(1064,800,128,44),'water',self.clear_water)
        self.button('Perspective',(1200,800,128,44),'camera')
        self.button('Top view',(1336,800,112,44),'top')
        self.button('Record',(1456,800,120,44),'record',self.recording)
        self.text(f'{len(lab.records)} live pings | {np.count_nonzero(lab.kept & lab.observed)} observed survivors | {np.count_nonzero(~lab.observed)} unobserved (hidden)',624,864,MUTED,self.mono)
        self.text(f'Sonar XYZ [{lab.position[0]:+.2f}, {lab.position[1]:+.2f}, {lab.position[2]:+.2f}] m | heading {lab.heading:+.1f} / tilt {lab.tilt:.1f} deg | 97 x 256 bins | top-hat beam',24,904,CYAN,self.mono)
        self.text('SPACE pause  W/A/S/D move boat  M target  P capture  E export  V record  F fullscreen  1/2 camera  Q quit',24,928,MUTED,self.mono)
        msg='Approximate, uncalibrated acoustics. Boat/water visual only. Fixed ROI; shadows conservative; no unique shape guarantee.'
        if lab.multipath: msg='MULTIPATH ON: artifacts enter carving too. No ghost removal; feasible volume may be biased.'
        self.text(msg,24,952,TRUTH if lab.multipath else MUTED,self.mono)
        self.text(self.notice or lab.status,24,976,MUTED,self.mono)
        self.screen.blit(pygame.transform.smoothscale(self.canvas,self.screen.get_size()),(0,0))
        pygame.display.flip()
        if self.recording and time.perf_counter()-self.last_record >= .1:
            frames=self.output/'frames';frames.mkdir(exist_ok=True)
            pygame.image.save(self.canvas,frames/f'{self.record_frames:04d}.png')
            self.record_frames+=1;self.last_record=time.perf_counter()
            if self.record_frames>=300:self.recording=False;self.notice='Recording cap reached: 300 PNG frames'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames',type=int,default=0)
    parser.add_argument('--screenshot')
    parser.add_argument('--fixed-dt',type=float,default=0)
    parser.add_argument('--size',default='1440x900')
    parser.add_argument('--target',choices=TARGETS,default='sphere')
    args=parser.parse_args()
    lab=Survey();lab.set_parameter('target',args.target)
    app=App(lab,tuple(map(int,args.size.split('x'))));clock=pygame.time.Clock();frame=0
    try:
        while not app.done:
            dt=args.fixed_dt or min(clock.tick(60)/1000,.1)
            for event in pygame.event.get():app.handle(event)
            lab.step(dt);app.draw();frame+=1
            if args.frames and frame>=args.frames:break
        if args.screenshot:
            path=Path(args.screenshot);path.parent.mkdir(parents=True,exist_ok=True)
            pygame.image.save(app.canvas,path)
        lab.export(app.output)
        print(json.dumps(dict(frames=frame,pings=lab.ping_number,output=str(app.output),render_ms=lab.render_ms)),flush=True)
    finally:pygame.quit()


if __name__=='__main__':main()
