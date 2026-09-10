"""Native software 3-D preview of the research model, not an Unreal rendering."""
from pathlib import Path
from datetime import datetime
import argparse, json, os, sys, time
ROOT=Path(__file__).resolve().parents[1]
os.chdir(ROOT)
sys.path[:0]=[os.environ.get('SONAR_CORE_DIR',str(ROOT)),str(ROOT/'python')]
import numpy as np
import pygame
import sonar, scene, mesh_assets, speckle

W,H=1600,1000
BG=(13,18,23); PANEL=(16,24,31); EDGE=(42,58,64); WHITE=(220,227,225); DIM=(139,159,164)
CYAN=(129,218,204); AMBER=(231,176,138)
NAMES=['Elevation ambiguity','Highlights & shadows','Surface multipath','Concave geometry']
LESSONS=['Move the target above and below. Its range and bearing stay the same.',
         'The nearest surface returns sound and hides surfaces behind it.',
         'A surface reflection travels farther and can create an extra return.',
         'Inspect the actual OBJ triangles that the acoustic core intersects.']

class Lab:
    def __init__(self):
        self.mode=1; self.roll=0.; self.gain=0.; self.subrays=256; self.frequency=600.; self.running=True
        self.noise=False; self.naive=False; self.multipath=False; self.orbit=-.65; self.elev=.44
        self.ping_number=0; self.t=0.; self.last_ping=-10.; self.render_ms=0.; self.error=None; self.sign=1
        self.position=np.array([0.,0.,0.]); self.history=[]; self.selected=48
        self.set_mode(1)
    def set_mode(self,mode):
        self.mode=mode;self.roll=0.;self.error=None;self.sign=1;self.t=0.;self.position[:]=0
        self.multipath=mode==2;self.noise=False;self.naive=False;self.history=[]
        if mode==2:self.position[2]=-.43
        self.build_scene();self.ping()
    def build_scene(self):
        mode=self.mode;self.objects=[];self.draw_meshes=[]
        self.floor=None if mode in (0,2) else -1.5
        if self.floor is not None:self.objects.append(sonar.make_plane((0,0,self.floor),(0,0,1),.05))
        if mode<3:
            self.centre=np.array(([0,4,self.sign*.35] if mode==0 else [0,4,-1.] if mode==1 else [0,2,-.15]),float)
            radius=[.16,.5,.08][mode]
            self.objects.append(sonar.make_sphere(self.centre,radius,.8))
            v,f=mesh_assets.radial_target(rings=16,segments=28)
            self.draw_meshes=[(v*radius*2+self.centre,f,np.array([155,134,109]))]
        else:
            self.centre=np.array([0,4,-.6])
            for name,scale,centre,color in [('concave_table',1.4,[0,4,-.6],[160,120,84]),('industrial_pipe',.7,[-1,5,-1],[113,143,143]),('coral_rock',1,[1.4,6,-1],[108,125,131])]:
                path=ROOT/'assets'/(name+'.obj');v=[];faces=[]
                for line in path.read_text().splitlines():
                    if line.startswith('v '):v.append(list(map(float,line.split()[1:4])))
                    elif line.startswith('f '):faces.append([int(s.split('/')[0])-1 for s in line.split()[1:4]])
                self.objects.append(sonar.make_mesh(str(path),scale=scale,translation=centre,reflectivity=.8))
                self.draw_meshes.append((np.array(v)*scale+centre,np.array(faces),np.array(color)))
    def ping(self):
        begin=time.perf_counter();self.axes=scene.tilted_axes(0,self.roll)
        self.sim=sonar.SonarSimulator(frequency_hz=self.frequency*1000,num_azimuth_bins=97,num_range_bins=256,
            max_range_m=10,horizontal_fov_deg=60,vertical_beamwidth_deg=30,num_elevation_subrays=1 if self.naive else self.subrays,
            beam_mode='top_hat',num_threads=4,multipath_enabled=self.multipath,surface_z=0)
        self.raw=self.sim.render(self.objects,self.position,self.axes)
        self.image=self.raw*speckle.factors(self.raw.shape,42+self.ping_number) if self.noise else self.raw.copy()
        self.ping_position=self.position.copy();self.ping_axes=self.axes.copy()
        self.render_ms=(time.perf_counter()-begin)*1000;self.ping_number+=1;self.last_ping=self.t
        self.history.append(self.raw.sum(axis=0));self.history=self.history[-80:]
    def flip(self):
        if self.mode!=0:self.set_mode(0)
        before=self.raw.copy();self.sign*=-1;self.build_scene();self.ping()
        self.error=float(np.max(np.abs(before-self.raw))/max(before.max(),1e-300))
    def step(self,dt):
        if not self.running:return
        self.t+=dt
        if self.t-self.last_ping>=1.6:self.ping()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--frames',type=int,default=0);parser.add_argument('--screenshot');parser.add_argument('--record');parser.add_argument('--fixed-dt',type=float,default=0);parser.add_argument('--mode',type=int,default=1);parser.add_argument('--size',default='1440x900');args=parser.parse_args()
    pygame.init();screen=pygame.display.set_mode(tuple(map(int,args.size.split('x'))),pygame.RESIZABLE)
    pygame.display.set_caption('Sonar Observatory | animated Mac research preview')
    canvas=pygame.Surface((W,H));clock=pygame.time.Clock()
    title=pygame.font.SysFont('Georgia',36);head=pygame.font.SysFont('Georgia',24);body=pygame.font.SysFont('Arial',17);mono=pygame.font.SysFont('Menlo',13)
    lab=Lab();lab.set_mode(args.mode);done=False;frame=0;click=None;drag=False;full=False;recorded=[]
    output=ROOT/'results'/('observatory_'+datetime.now().strftime('%Y-%m-%d_%H%M%S_%f'));output.mkdir(parents=True)
    view=pygame.Rect(264,304,856,464);sonar_rect=pygame.Rect(1160,304,416,436)
    yy,xx=np.mgrid[:sonar_rect.h,:sonar_rect.w];dx=xx-sonar_rect.w/2;dy=sonar_rect.h-24-yy
    rr=np.hypot(dx,dy);theta=np.arctan2(dx,dy);radius=sonar_rect.h-64
    valid=(np.abs(theta)<=np.pi/6)&(rr<radius)&(dy>=0)
    az=np.clip(((theta/(np.pi/3)+.5)*97).astype(int),0,96);rb=np.clip((rr/radius*256).astype(int),0,255)
    def text(s,x,y,color=WHITE,font=body):canvas.blit(font.render(str(s),True,color),(x,y))
    def button(label,x,y,w,active=False):
        rect=pygame.Rect(x,y,w,40);hover=rect.collidepoint(mouse)
        pygame.draw.rect(canvas,(35,60,61) if active else (25,35,43) if hover else PANEL,rect,border_radius=3)
        pygame.draw.rect(canvas,CYAN if active else EDGE,rect,1,border_radius=3)
        text(label,x+12,y+11,CYAN if active else WHITE)
        return click is not None and rect.collidepoint(click)
    def line3(a,b,color,width=1):
        pts,depth=project(np.array([a,b],float))
        if np.all(depth>.1):pygame.draw.line(canvas,color,pts[0],pts[1],width)
    started=time.perf_counter()
    print('Loaded acoustic extension:',sonar.__file__,flush=True)
    while not done:
        dt=args.fixed_dt or min(clock.tick(60)/1000,.1);click=None
        mouse=tuple(np.array(pygame.mouse.get_pos())*[W/screen.get_width(),H/screen.get_height()])
        for e in pygame.event.get():
            if e.type==pygame.QUIT:done=True
            elif e.type==pygame.VIDEORESIZE and not full:screen=pygame.display.set_mode((max(1000,e.w),max(625,e.h)),pygame.RESIZABLE)
            elif e.type==pygame.MOUSEBUTTONDOWN:
                if e.button==1:click=tuple(np.array(e.pos)*[W/screen.get_width(),H/screen.get_height()])
                if e.button==3:drag=True
            elif e.type==pygame.MOUSEBUTTONUP and e.button==3:drag=False
            elif e.type==pygame.MOUSEMOTION and drag:
                lab.orbit+=e.rel[0]*.005;lab.elev=np.clip(lab.elev+e.rel[1]*.004,.12,1.2)
            elif e.type==pygame.KEYDOWN:
                if e.key in (pygame.K_ESCAPE,pygame.K_q):done=True
                elif e.key==pygame.K_SPACE:lab.running=not lab.running
                elif e.key==pygame.K_RETURN:lab.ping()
                elif e.key==pygame.K_r:lab.set_mode(lab.mode)
                elif e.key==pygame.K_e:lab.flip()
                elif e.key in (pygame.K_1,pygame.K_2,pygame.K_3,pygame.K_4):lab.set_mode(e.key-pygame.K_1)
                elif e.key==pygame.K_f:
                    full=not full;screen=pygame.display.set_mode((0,0) if full else (1440,900),pygame.FULLSCREEN if full else pygame.RESIZABLE)
                elif e.key==pygame.K_p:pygame.image.save(canvas,output/f'capture_{frame}.png')
        keys=pygame.key.get_pressed()
        if lab.running:
            lab.roll+=(keys[pygame.K_RIGHT]-keys[pygame.K_LEFT])*30*dt
            lab.position[0]+=(keys[pygame.K_d]-keys[pygame.K_a])*.5*dt
            lab.position[2]+=(keys[pygame.K_w]-keys[pygame.K_s])*.5*dt
        lab.step(dt);phase=np.clip((lab.t-lab.last_ping)/1.2,0,1)
        if not lab.running:phase=1
        canvas.fill(BG)
        text('S O N A R',24,24,CYAN,body);text('O B S E R V A T O R Y',24,48,DIM,mono)
        text('The laboratory',300,32);text('MAC RESEARCH PREVIEW  /  UNREAL BUILD PENDING',1016,34,DIM,mono)
        pygame.draw.line(canvas,EDGE,(24,82),(1576,82))
        text('Watch sound reveal a scene.',24,104,WHITE,title)
        text('Real C++ measurements. An animated window into the model.',856,120,DIM)
        pygame.draw.rect(canvas,(25,29,32),(24,168,1552,64),border_radius=4)
        text(f'0{lab.mode+1}',44,188,AMBER,head);text(LESSONS[lab.mode],104,190,WHITE)
        text('01  THE EXPERIMENT',24,266,DIM,mono)
        for i,name in enumerate(NAMES):
            if button(name,24,304+i*52,208,i==lab.mode):lab.set_mode(i)
        text('02  THE ACOUSTICS',24,536,DIM,mono)
        if button('One ray' if lab.naive else 'Elevation integrated',24,572,208,not lab.naive):lab.naive=not lab.naive;lab.ping()
        if button('Multipath on' if lab.multipath else 'Multipath off',24,624,208,lab.multipath):lab.multipath=not lab.multipath;lab.ping()
        if button('Speckle on' if lab.noise else 'Speckle off',24,676,208,lab.noise):lab.noise=not lab.noise;lab.ping()
        if button('Flip elevation',24,752,208):lab.flip()
        if button('Gain -',24,804,100):lab.gain=max(-20,lab.gain-3)
        if button('Gain +',132,804,100):lab.gain=min(20,lab.gain+3)
        text(f'GAIN {lab.gain:+.0f} dB',24,858,DIM,mono)
        text('03  THE PHYSICAL SCENE',264,266,DIM,mono)
        if button('Pause' if lab.running else 'Run',802,252,92,lab.running):lab.running=not lab.running
        if button('Step',902,252,92):lab.ping()
        if button('Reset',1002,252,118):lab.set_mode(lab.mode)
        pygame.draw.rect(canvas,(9,24,32),view)
        canvas.set_clip(view)
        for row in range(view.h):
            t=row/view.h;pygame.draw.line(canvas,(int(8+7*t),int(25+9*t),int(35+8*t)),(view.x,view.y+row),(view.right,view.y+row))
        focus=np.array([0,3.5,-.5]);angle=lab.orbit+.045*np.sin(lab.t*.25)
        eye=focus+np.array([8*np.sin(angle),-8*np.cos(angle),8*np.sin(lab.elev)])
        forward=(focus-eye);forward/=np.linalg.norm(forward);right=np.cross(forward,[0,0,1]);right/=np.linalg.norm(right);up=np.cross(right,forward)
        def project(points):
            d=points-eye;depth=np.sum(d*forward,axis=1)
            xy=np.stack([view.centerx+620*np.sum(d*right,axis=1)/np.maximum(depth,.1),view.centery-620*np.sum(d*up,axis=1)/np.maximum(depth,.1)],axis=1)
            return np.clip(xy,-10000,10000).astype(int),depth
        floor=lab.floor if lab.floor is not None else -1.8
        for y in np.arange(-1,12,1.):line3([-4,y,floor],[4,y,floor],(28,57,66))
        for x in np.arange(-4,5,1.):line3([x,-1,floor],[x,11,floor],(28,57,66))
        polys=[]
        for v,f,color in lab.draw_meshes:
            xy,depth=project(v)
            for tri in f:
                if np.any(depth[tri]<.1):continue
                normal=np.cross(v[tri[1]]-v[tri[0]],v[tri[2]]-v[tri[0]]);normal/=max(np.linalg.norm(normal),1e-8)
                lighting=.25+.75*abs(np.sum(normal*np.array([-.4,-.5,.76])))
                polys.append((depth[tri].mean(),xy[tri],tuple(np.clip(color*lighting,0,255).astype(int))))
        for depth,poly,color in sorted(polys,key=lambda p:-p[0]):pygame.draw.polygon(canvas,color,poly)
        p=lab.ping_position;axes=lab.ping_axes
        for phi in [-15,15]:
            rays=[]
            for theta_deg in np.linspace(-30,30,41):
                direction=np.sum(axes*scene.spherical_to_world(1,theta_deg,phi)[None,:],axis=1)
                rays.append(p+direction*8)
            for a,b in zip(rays,rays[1:]):line3(a,b,(38,92,91))
            for q in (rays[0],rays[-1]):line3(p,q,(38,92,91))
        scan=[p+np.sum(axes*scene.spherical_to_world(10*phase,a,0)[None,:],axis=1) for a in np.linspace(-30,30,60)]
        if phase<1:
            for a,b in zip(scan,scan[1:]):line3(a,b,CYAN,2)
        if lab.mode<3:
            line3(p,lab.centre,CYAN,2)
            if lab.multipath:
                mirror=p*np.array([1,1,-1]);t=-lab.centre[2]/(mirror[2]-lab.centre[2]) if abs(mirror[2]-lab.centre[2])>1e-9 else 0
                bounce=lab.centre+(mirror-lab.centre)*t
                line3(p,bounce,AMBER,2);line3(bounce,lab.centre,AMBER,2)
        xy,_=project(np.array([lab.position]));pygame.draw.circle(canvas,(213,223,214),xy[0],9);pygame.draw.circle(canvas,CYAN,xy[0],13,1)
        rng=np.random.default_rng(42)
        particles=np.column_stack([rng.uniform(-3,3,90),rng.uniform(-1,10,90),rng.uniform(-1.5,1.5,90)])
        particles[:,0]+=.04*np.sin(lab.t+particles[:,1]);px,pd=project(particles)
        for q,z in zip(px,pd):
            if z>.1:pygame.draw.circle(canvas,(56,83,90),q,1)
        canvas.set_clip(None)
        text('Right-drag to orbit. Camera motion never changes the sonar.',280,732,DIM,mono)
        text('04  RANGE + BEARING',1160,266,DIM,mono)
        level=np.clip((scene.to_decibels(lab.image)+lab.gain+60)/60,0,1)
        values=level[az,rb];values=np.where(valid & (rr/radius<=phase),values,0)
        rgb=np.stack([10+values*145,23+values*218,27+values*182],axis=2).astype(np.uint8)
        surf=pygame.surfarray.make_surface(np.transpose(rgb,(1,0,2)));canvas.blit(surf,sonar_rect)
        origin=(sonar_rect.centerx,sonar_rect.bottom-24)
        for ring in range(1,5):
            pts=[(origin[0]+np.sin(a)*radius*ring/4,origin[1]-np.cos(a)*radius*ring/4) for a in np.linspace(-np.pi/6,np.pi/6,60)]
            pygame.draw.lines(canvas,(54,80,83),False,pts,1);text(f'{ring*2.5:g} m',origin[0]+8,origin[1]-radius*ring/4,DIM,mono)
        pygame.draw.polygon(canvas,CYAN,[(origin[0],origin[1]-5),(origin[0]-6,origin[1]+7),(origin[0]+6,origin[1]+7)])
        if click and sonar_rect.collidepoint(click):lab.selected=int(np.clip((np.arctan2(click[0]-origin[0],origin[1]-click[1])/(np.pi/3)+.5)*97,0,96))
        text(f'{lab.frequency:.0f} kHz / {1 if lab.naive else lab.subrays} elevation rays',1160,752,DIM,mono)
        text('SELECTED BEARING / POWER',1160,796,DIM,mono)
        trace=scene.to_decibels(lab.raw[lab.selected],reference=max(lab.raw.max(),1e-300))
        pts=[(1160+i*416/255,884-(v+60)/60*64) for i,v in enumerate(trace)]
        pygame.draw.lines(canvas,CYAN,False,pts,2)
        text(f'PING {lab.ping_number:04d}',264,800,CYAN,mono);text(f'{lab.render_ms:.1f} ms / acoustic render',430,800,DIM,mono)
        text(f'ROLL {lab.roll:+.1f} deg',264,832,DIM,mono);text(f'RAW PEAK {lab.raw.max():.3e}',520,832,DIM,mono)
        if lab.error is not None:text(f'+/- elevation relative difference: {lab.error:.3e}',264,868,AMBER)
        else:text('Elevation collapses into each range-bearing bin.',264,868,WHITE,head)
        pygame.draw.line(canvas,EDGE,(24,918),(1576,918))
        text('SPACE pause / ENTER ping / 1-4 experiment / A,D sideways / W,S vertical / arrows roll / P capture / Q quit',24,938,DIM,mono)
        text('Approximate scattering + multipath. No ghost removal. Sweep slowed for inspection. Particles are decorative.',24,966,DIM,mono)
        screen.blit(pygame.transform.smoothscale(canvas,screen.get_size()),(0,0));pygame.display.flip();frame+=1
        if args.record and frame%3==0:
            from PIL import Image
            recorded.append(Image.frombytes('RGB',(W,H),pygame.image.tostring(canvas,'RGB')).resize((1120,700)))
        if args.frames and frame>=args.frames:done=True
    if args.screenshot:pygame.image.save(canvas,args.screenshot)
    if args.record and recorded:recorded[0].save(args.record,save_all=True,append_images=recorded[1:],duration=100,loop=0)
    report=dict(frames=frame,pings=lab.ping_number,acoustic_extension=sonar.__file__,mode=lab.mode,
        raw_peak=float(lab.raw.max()),gain_db=lab.gain,speckle=lab.noise,multipath=lab.multipath,
        selected_bearing=lab.selected,elevation_error=lab.error,render_ms=lab.render_ms,config={'bearings':97,'ranges':256,'subrays':lab.subrays,'threads':4},
        implementation='Pygame perspective triangle preview, not Unreal',seconds=lab.t,
        wall_seconds=time.perf_counter()-started,measured_loop_fps=frame/max(time.perf_counter()-started,1e-9))
    (output/'session.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True);pygame.quit()
if __name__=='__main__':main()
