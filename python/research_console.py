"""Inspectable marine research console. Presentation geometry never alters measurements."""
import argparse,json,time,sys
from pathlib import Path
from datetime import datetime
sys.path[:0]=['.','python']
import numpy as np
import pygame
import sonar,scene as sc,display as disp,mesh_assets,reconstruction as rc,speckle
from console import Slider,Toggle,surface_from,polyline

W,H=1600,1000
BG=(5,14,20);PANEL=(9,25,34);EDGE=(37,66,76);TEXT=(196,222,225);DIM=(112,150,158)
GREEN=(67,244,143);AMBER=(255,188,70);BLUE=(75,177,255);PINK=(245,104,163)
PARAMETERS=[
 ('sonar X',-2,2,0),('sonar Y',-2,2,0),('sonar Z',-2,-.1,-.6),('yaw',-50,50,0),('tilt',-20,40,14),('roll',-90,90,20),('surge m/s',0,1,.2),('yaw rate',-20,20,0),
 ('target X',-2,2,0),('target Y',2,8,4),('target Z',-2.4,-.3,-1.3),('scale',.2,1.4,.7),('target yaw',-180,180,0),('reflectivity',.05,1,.8),('orbit',-180,180,-35),('voxel m',.06,.2,.1),
 ('frequency kHz',100,2000,900),('beam deg',4,50,30),('FOV deg',15,90,40),('sub-rays',32,4096,1024),('gain dB',-20,20,0),('dynamic dB',20,90,60),('elements',4,96,64),('spacing/lambda',.25,1,.5),
 ('surface Z',-.05,1,0),('roughness mm',0,1,0),('multipath amp',0,1,1),('correlation',0,4,0),('turbidity',.02,1.2,.1),('poses',4,32,8),('threshold',.001,.2,.03),('bandwidth kHz',10,150,60)]
TOGGLES=['direct','shadow mask','ghost','mirror','multipath','speckle','texture','reconstruction','detections','animation','optical']
MODELS=['sphere','box','coral_rock','industrial_pipe','concave_table']


def rotation(yaw,tilt=0,roll=0):
    a=np.radians(yaw);rz=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
    return rz@sc.tilted_axes(tilt,roll)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--frames',type=int,default=0);parser.add_argument('--size',default='1440x900');parser.add_argument('--screenshot');parser.add_argument('--reconstruct',action='store_true');parser.add_argument('--animate',action='store_true');args=parser.parse_args()
    pygame.init();screen=pygame.display.set_mode(tuple(map(int,args.size.split('x'))),pygame.RESIZABLE)
    pygame.display.set_caption('FSS Research | Forward-scan sonar laboratory')
    canvas=pygame.Surface((W,H));clock=pygame.time.Clock();font=pygame.font.SysFont('Menlo',15);small=pygame.font.SysFont('Menlo',12);title=pygame.font.SysFont('Menlo',21,bold=True)
    def text(s,x,y,color=TEXT,f=font):canvas.blit(f.render(str(s),True,color),(x,y))
    def panel(rect,label):
        pygame.draw.rect(canvas,PANEL,rect);pygame.draw.rect(canvas,EDGE,rect,1);text(label,rect.x+12,rect.y+9,DIM,small)
    rects={k:pygame.Rect(*v) for k,v in dict(scene=(20,90,400,260),sonar=(450,90,400,540),beam=(880,90,380,150),
       optical=(880,260,380,170),ascan=(880,450,380,180),recon=(20,370,400,260),btr=(20,675,610,155),rtr=(650,675,610,155)).items()}
    sliders=[Slider(pygame.Rect(1420,170+(i%8)*43,100,7),name,lo,hi,value,'{:.2f}') for i,(name,lo,hi,value) in enumerate(PARAMETERS)]
    values={s.label:s for s in sliders};toggles={name: name in ('direct','ghost','mirror','multipath','optical') for name in TOGGLES}
    def val(name):return values[name].value
    page=0;held=None;model=0;beam_mode=0;full=False;running=True;frame=0;ping=0;advance=0.;heading=0.;last_key=None
    btr=np.full((100,128),-60.);rtr=np.full((100,320),-60.);kept=None;points=None;recording=False;recorded=[];screenshot=False
    assets=mesh_assets.generate();last_time=time.perf_counter();render_ms=0.;times=[];camera_key=None
    session_start=time.perf_counter()
    if args.animate:toggles['animation']=True
    if args.reconstruct:pygame.event.post(pygame.event.Event(pygame.KEYDOWN,key=pygame.K_k))
    capture_dir=Path('results')/('console_'+datetime.now().strftime('%Y-%m-%d_%H%M%S_%f'));capture_dir.mkdir(parents=True)
    while running:
        now=time.perf_counter();dt=now-last_time;last_time=now
        for event in pygame.event.get():
            if event.type==pygame.QUIT:running=False
            if event.type==pygame.VIDEORESIZE and not full:screen=pygame.display.set_mode((max(1000,event.w),max(625,event.h)),pygame.RESIZABLE)
            if event.type==pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE,pygame.K_q):running=False
                elif event.key==pygame.K_SPACE:toggles['animation']=not toggles['animation']
                elif event.key==pygame.K_f:
                    full=not full;screen=pygame.display.set_mode((0,0) if full else (1440,900),pygame.FULLSCREEN if full else pygame.RESIZABLE)
                elif event.key==pygame.K_r:
                    for s,spec in zip(sliders,PARAMETERS):s.value=spec[3]
                    advance=0;heading=0;last_key=None;kept=None;btr.fill(-60);rtr.fill(-60)
                elif event.key==pygame.K_p:screenshot=True
                elif event.key==pygame.K_v:
                    recording=not recording
                    if not recording and recorded:
                        recorded[0].save(capture_dir/'recording.gif',save_all=True,append_images=recorded[1:],duration=100,loop=0);recorded=[]
                elif event.key==pygame.K_m:model=(model+1)%len(MODELS);kept=None
                elif event.key==pygame.K_b:beam_mode=(beam_mode+1)%3
                elif event.key==pygame.K_TAB:page=(page+1)%4
                elif event.key==pygame.K_k:
                    centre=np.array([val('target X'),val('target Y'),val('target Z')]);points,shape=rc.grid(centre,half=val('scale'),step=val('voxel m'))
                    poses=list(rc.orbit_poses(int(val('poses')),centre))
                    scan=sonar.SonarSimulator(frequency_hz=val('frequency kHz')*1000,num_azimuth_bins=65,num_range_bins=220,
                        num_elevation_subrays=int(val('sub-rays')),beam_mode=['array','top_hat','hann'][beam_mode],
                        vertical_beamwidth_deg=val('beam deg'),horizontal_fov_deg=val('FOV deg'),max_range_m=10,num_threads=1)
                    target=([sonar.make_sphere(centre,val('scale')/2,val('reflectivity'))] if model==0 else
                            [sonar.make_mesh('assets/'+MODELS[model]+'.obj',scale=val('scale'),translation=centre,reflectivity=val('reflectivity'),axes=rotation(val('target yaw')))])
                    ims=[scan.render(target,p,a) for p,a in poses]
                    if toggles['speckle']:
                        ims=[im*speckle.factors(im.shape,500+j,(val('correlation'),val('correlation'))) for j,im in enumerate(ims)]
                    kept=rc.reconstruct(ims,poses,scan,points,val('threshold'));toggles['reconstruction']=True
                    np.savez_compressed(capture_dir/'carving.npz',points=points,kept=kept,images=np.array(ims),positions=np.array([p for p,a in poses]),axes=np.array([a for p,a in poses]))
            if event.type in (pygame.MOUSEBUTTONDOWN,pygame.MOUSEMOTION,pygame.MOUSEBUTTONUP):
                pos=(event.pos[0]*W/screen.get_width(),event.pos[1]*H/screen.get_height())
                if event.type==pygame.MOUSEBUTTONDOWN:
                    if 1290<=pos[0]<=1580 and 112<=pos[1]<=144:page=min(3,int((pos[0]-1290)/73))
                    for s in sliders[page*8:page*8+8]:
                        if s.hit(pos):held=s;s.drag(pos);kept=None
                    for i,name in enumerate(TOGGLES):
                        if pygame.Rect(1300,545+i*25,280,22).collidepoint(pos):toggles[name]=not toggles[name]
                elif event.type==pygame.MOUSEBUTTONUP:held=None
                elif held is not None:held.drag(pos);kept=None
        if toggles['animation']:
            advance+=val('surge m/s')*dt;heading+=val('yaw rate')*dt
            if advance>2:advance=0
        display_only={'gain dB','dynamic dB','orbit','voxel m','poses','threshold','bandwidth kHz','turbidity'}
        key=(tuple(s.value for s in sliders if s.label not in display_only),
             tuple((name,state) for name,state in toggles.items() if name not in ('shadow mask','reconstruction','detections','optical')),model,beam_mode,advance,heading)
        new_ping=key!=last_key
        if new_ping:
            start=time.perf_counter();ping+=1;last_key=key
            centre=np.array([val('target X'),val('target Y'),val('target Z')]);position=np.array([val('sonar X'),val('sonar Y')+advance,val('sonar Z')]);axes=rotation(val('yaw')+heading,val('tilt'),val('roll'))
            target=sonar.make_sphere(centre,val('scale')/2,val('reflectivity'),texture_amplitude=.4 if toggles['texture'] else 0) if model==0 else sonar.make_mesh('assets/'+MODELS[model]+'.obj',scale=val('scale'),translation=centre,reflectivity=val('reflectivity'),axes=rotation(val('target yaw')))
            objects=[sonar.make_plane((0,0,-2.6),(0,0,1),.06),target]
            frequency=val('frequency kHz')*1000
            cfg=dict(frequency_hz=frequency,num_azimuth_bins=128,num_range_bins=320,max_range_m=10,num_elevation_subrays=int(val('sub-rays')),
                horizontal_fov_deg=val('FOV deg'),vertical_beamwidth_deg=val('beam deg'),array_element_count=int(val('elements')),
                array_element_spacing_m=val('spacing/lambda')*1500/frequency,beam_mode=['array','top_hat','hann'][beam_mode],num_threads=1,
                multipath_enabled=toggles['multipath'],surface_z=val('surface Z'),surface_rms_height_m=val('roughness mm')/1000,surface_reflectivity=val('multipath amp'),
                direct_enabled=toggles['direct'],ghost_enabled=toggles['ghost'],mirror_enabled=toggles['mirror'])
            sim=sonar.SonarSimulator(**cfg);raw=sim.render(objects,position,axes)
            image=raw*speckle.factors(raw.shape,ping,(val('correlation'),val('correlation'))) if toggles['speckle'] else raw.copy()
            angles=np.linspace(-val('beam deg')/2,val('beam deg')/2,201)
            pattern=np.array([sonar.beam_response(np.radians(a),int(val('elements')),val('spacing/lambda'),1.,['array','top_hat','hann'][beam_mode]) for a in angles])
            render_ms=(time.perf_counter()-start)*1000;times.append(render_ms)
        decibels=sc.to_decibels(image,floor_db=-val('dynamic dB'))+val('gain dB')
        bearing=sc.to_decibels(image.sum(axis=1));trace=sc.to_decibels(image[64],reference=max(image.max(),1e-30))
        if new_ping:
            btr[1:]=btr[:-1];btr[0]=bearing;rtr[1:]=rtr[:-1];rtr[0]=trace
        new_camera_key=(key,val('turbidity'),toggles['optical'])
        if new_camera_key!=camera_key:
            camera_key=new_camera_key
            optical=sonar.OpticalCamera(width=240,height=120,focal_px=160,position=position+[0,0,.25],axes=axes,
                attenuation_per_m=val('turbidity')).render(objects) if toggles['optical'] else np.zeros((120,240))
        canvas.fill(BG)
        text('FSS / RESEARCH LABORATORY',20,18,GREEN,title)
        text('Range + bearing measurements | elevation is integrated out',20,49,DIM)
        text(f'{MODELS[model]}  |  ping {ping:04d}  |  {render_ms:.1f} ms/ping  |  {clock.get_fps():.0f} display fps',790,23)
        text('SPACE run/pause   F fullscreen   R reset   P capture   V record   Q quit',790,49,DIM,small)
        for name,rect in rects.items():panel(rect,dict(scene='01 / 3-D SCENE | metres',sonar='02 / RANGE-BEARING',beam='03 / BEAM POWER',optical='04 / OPTICAL | approximate',ascan='05 / CENTRE-BEARING A-SCAN',recon='06 / FEASIBLE VOLUME',btr='07 / BEARING-TIME | latest ping at top',rtr='08 / RANGE-TIME | latest ping at top')[name])
        def pixels(im,rect):canvas.blit(pygame.transform.scale(surface_from(im),rect.size),rect)
        rr=rects['sonar'].inflate(-52,-70);rr.y+=8
        rgb=disp.to_pixels(decibels.T,floor=-val('dynamic dB'))
        if toggles['shadow mask']:
            hi,sh=rc.segment(raw,val('threshold'));rgb=np.zeros((*raw.T.shape,3),np.uint8);rgb[hi.T]=GREEN;rgb[sh.T]=AMBER
        pixels(rgb,rr)
        for r in range(0,11,2):text(str(r)+'m',rr.right+3,rr.y+rr.height*r/10-6,DIM,small)
        text(f"{-val('FOV deg')/2:.0f} deg        0        {val('FOV deg')/2:.0f} deg",rr.x,rr.bottom+10,DIM,small)
        if toggles['detections']:
            for a in range(128):
                r=int(np.argmax(image[a]))
                if image[a,r]>.1*image.max():pygame.draw.circle(canvas,AMBER,(rr.x+a*rr.w//128,rr.y+r*rr.h//320),2)
        for name,data in [('btr',btr),('rtr',rtr)]:pixels(disp.to_pixels(data),rects[name].inflate(-24,-48).move(0,10))
        pixels(np.repeat((np.clip(optical,0,1)**.45*255).astype(np.uint8)[...,None],3,axis=2),rects['optical'].inflate(-24,-48).move(0,10))
        polyline(canvas,rects['beam'].inflate(-24,-65).move(0,10),10*np.log10(np.maximum(pattern,1e-8)),-60,0,GREEN,2)
        text(f"{['uniform array','top-hat','Hann array'][beam_mode]} | B(0)=1 | -60 to 0 dB",892,211,DIM,small)
        polyline(canvas,rects['ascan'].inflate(-24,-80).move(0,0),trace,-60,0,GREEN,2);text('0 m',893,602,DIM,small);text('10 m',1218,602,DIM,small)
        # Orthographic 3-D inspection view, explicitly separate from sonar projection.
        orbit=np.radians(val('orbit'));view_right=np.array([np.cos(orbit),np.sin(orbit),0]);view_up=np.array([-.35*np.sin(orbit),.35*np.cos(orbit),.94])
        bounds=np.array([[x,y,z] for x in (-3,3) for y in (0,10) for z in (-2.6,val('surface Z'))]+[position.tolist(),centre.tolist()])
        horizontal=bounds@view_right;vertical=bounds@view_up
        mid_x=(horizontal.min()+horizontal.max())/2;mid_y=(vertical.min()+vertical.max())/2
        zoom=min(360/np.ptp(horizontal),195/np.ptp(vertical))
        def project(p,rect):
            p=np.asarray(p)
            return (int(rect.centerx+zoom*(np.sum(p*view_right)-mid_x)),int(rect.centery+15-zoom*(np.sum(p*view_up)-mid_y)))
        scene_rect=rects['scene'];canvas.set_clip(scene_rect.inflate(-4,-35).move(0,15))
        def line(a,b,color,width=1):pygame.draw.line(canvas,color,project(a,scene_rect),project(b,scene_rect),width)
        for y in np.arange(0,11,1):line((-3,y,-2.6),(3,y,-2.6),EDGE)
        for x in np.arange(-3,4,1):line((x,0,-2.6),(x,10,-2.6),EDGE)
        wave_time=now if toggles['animation'] else 0
        for x in (-3,0,3):
            ps=[(x,y,val('surface Z')+.04*np.sin(y*3+wave_time)) for y in np.linspace(0,10,35)]
            pygame.draw.lines(canvas,(28,96,120),False,[project(p,scene_rect) for p in ps])
        verts,faces=assets[MODELS[model]];world=verts*val('scale')@rotation(val('target yaw')).T+centre
        for face in faces:
            pygame.draw.lines(canvas,DIM,True,[project(p,scene_rect) for p in world[face]])
        for theta in (-val('FOV deg')/2,val('FOV deg')/2):
            for phi in (-val('beam deg')/2,val('beam deg')/2):line(position,position+axes@sc.spherical_to_world(8,theta,phi),(30,87,54))
        housing_vertices,housing_faces=mesh_assets.box((-.12,-.2,-.08),(.12,.1,.08))
        housing_vertices=housing_vertices@axes.T+position
        for face in housing_faces:
            pygame.draw.lines(canvas,GREEN,True,[project(p,scene_rect) for p in housing_vertices[face]])
        for particle in np.random.default_rng(10).uniform([-3,0,-2.5],[3,9,-.1],(25,3)):
            pygame.draw.circle(canvas,(47,73,83),project(particle,scene_rect),1)
        if toggles['animation']:
            ping_range=(now%2)*4
            arc=[position+axes@sc.spherical_to_world(ping_range,t,0) for t in np.linspace(-val('FOV deg')/2,val('FOV deg')/2,25)]
            pygame.draw.lines(canvas,GREEN,False,[project(p,scene_rect) for p in arc])
        if model==0 and toggles['shadow mask']:
            offset=centre-position;distance=np.linalg.norm(offset);forward=offset/distance
            right=np.cross(forward,[0,0,1]);right/=np.linalg.norm(right);up=np.cross(right,forward)
            half=np.arcsin(min(.99,val('scale')/(2*distance)))
            for a in np.linspace(0,2*np.pi,20,endpoint=False):
                direction=forward*np.cos(half)+(right*np.cos(a)+up*np.sin(a))*np.sin(half)
                line(position+direction*distance,position+direction*10,AMBER)
        line(position,centre,GREEN,2)
        mirrored=centre.copy();mirrored[2]=2*val('surface Z')-centre[2];line(position,mirrored,BLUE)
        virtual=position.copy();virtual[2]=2*val('surface Z')-position[2]
        if virtual[2]!=centre[2]:
            bounce=centre+(virtual-centre)*(val('surface Z')-centre[2])/(virtual[2]-centre[2]);line(centre,bounce,PINK);line(bounce,position,PINK)
        pygame.draw.circle(canvas,GREEN,project(position,scene_rect),5)
        canvas.set_clip(None);text('green direct / pink ghost / blue mirror',32,326,DIM,small)
        if kept is not None and toggles['reconstruction']:
            recon_rect=rects['recon'];canvas.set_clip(recon_rect.inflate(-10,-40).move(0,15))
            for p in points[kept][::max(1,int(kept.sum()/2500))]:
                offset=p-centre;loc=(int(recon_rect.centerx+120*np.sum(offset*view_right)),int(recon_rect.centery+120*np.sum(-offset*view_up)))
                pygame.draw.circle(canvas,GREEN,loc,2)
            canvas.set_clip(None);text(f'{kept.sum()} / {len(kept)} voxels; direct orbit',32,603,DIM,small)
        else:
            text('K: acquire known poses and carve',38,475)
            text('Highlight + feasible-shadow consistency',38,506,DIM,small)
            text('Target-only direct orbit; no truth input',38,530,DIM,small)
        panel(pygame.Rect(1290,90,290,860),'PARAMETERS / drag to change')
        for i,name in enumerate(('POSE','TARGET','SONAR','MEDIUM')):
            pygame.draw.rect(canvas,GREEN if i==page else EDGE,(1294+i*71,119,67,23));text(name,1299+i*71,123,BG if i==page else TEXT,small)
        for s in sliders[page*8:page*8+8]:s.draw(canvas,small)
        text('M model: '+MODELS[model],1300,512,GREEN,small)
        for i,(name,state) in enumerate(toggles.items()):text(('[x] ' if state else '[ ] ')+name,1300,546+i*25,GREEN if state else DIM,small)
        text('B beam mode | TAB page | K carve',1300,840,DIM,small)
        text('128 bearings x 320 ranges',1300,869,DIM,small);text('1 CPU thread; midpoint integration',1300,891,DIM,small)
        text('REC (10 fps; 300-frame cap)' if recording else 'Paused pings are cached',1300,919,AMBER if recording else DIM,small)
        rd=np.linalg.norm(centre-position);rm=np.linalg.norm(mirrored-position)
        text(f'Wavelength {sim.wavelength_m*1000:.3f} mm   |   Thorp {sonar.thorp_alpha(frequency):.1f} dB/km   |   c/(2B) {1500/(2*val("bandwidth kHz")):.2f} mm',20,852,GREEN)
        text(f'Centre-path ranges: direct {rd:.3f} m   ghost {(rd+rm)/2:.3f} m   mirror {rm:.3f} m | surface peaks differ',20,879,TEXT)
        text('ASSUMPTIONS: ideal array; diffuse patches; angular mean; point-scatterer r^-4; planar coherent multipath.',20,909,DIM,small)
        text('No DIDSON calibration, ghost removal, concavity reverberation or equation (6). Feasible volume is not unique.',20,931,DIM,small)
        text('Water, particles and slow ping are presentation only. Bandwidth is waveform theory; image bins have no pulse convolution.',20,953,DIM,small)
        screen.blit(pygame.transform.smoothscale(canvas,screen.get_size()),(0,0));pygame.display.flip();frame+=1
        if screenshot or (args.frames and frame==args.frames):
            path=Path(args.screenshot) if args.screenshot else capture_dir/f'frame_{frame:06d}.png';path.parent.mkdir(parents=True,exist_ok=True);pygame.image.save(screen,str(path));screenshot=False
        if recording and frame%6==0 and len(recorded)<300:
            from PIL import Image
            recorded.append(Image.frombytes('RGB',(W,H),pygame.image.tobytes(canvas,'RGB')).resize((960,600)))
        if args.frames and frame>=args.frames:running=False
        clock.tick(60)
    if recorded:recorded[0].save(capture_dir/'recording.gif',save_all=True,append_images=recorded[1:],duration=100,loop=0)
    (capture_dir/'session.json').write_text(json.dumps(dict(pings=ping,display_frames=frame,elapsed_s=time.perf_counter()-session_start,display_fps=frame/(time.perf_counter()-session_start),render_ms=times,config=cfg,parameters={s.label:s.value for s in sliders},
        approximations='See docs/scientific_status.md'),indent=2))
    print(f'Pygame completed {frame} frames, {ping} pings; session {capture_dir}');pygame.quit()

if __name__=='__main__':main()
