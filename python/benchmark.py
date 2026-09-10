"""Measured stage timings with full configurations; no bare FPS claim."""
import sys,time,json,platform
sys.path[:0]=['.','python']
import numpy as np
import sonar,scene as sc,speckle,outputs

records=[]
for target,triangles in [('analytic',0),('coral_rock',360)]:
    objects=[sonar.make_plane((0,0,-2),(0,0,1),.06)]
    objects += [sonar.make_sphere((0,4,-1.3),.35,.8)] if target=='analytic' else [sonar.make_mesh('assets/coral_rock.obj',scale=.7,translation=(0,4,-1.3))]
    for threads in (1,8):
        cfg=dict(num_azimuth_bins=97,num_range_bins=400,num_elevation_subrays=512,num_threads=threads,
                 frequency_hz=900e3,vertical_beamwidth_deg=30,max_range_m=9,multipath_enabled=True)
        sim=sonar.SonarSimulator(**cfg);axes=sc.tilted_axes(14,35)
        camera=sonar.OpticalCamera(width=160,height=120,focal_px=125,position=(0,0,-.6),axes=axes,attenuation_per_m=.1)
        timing={name:[] for name in ('Cython_scene_and_CPP_render_ms','correlated_speckle_ms','optical_ms','log_display_ms')}
        for index in range(22):
            start=time.perf_counter();image=sim.render(objects,position=(0,0,-.6),axes=axes);rendered=time.perf_counter()
            noisy=image*speckle.factors(image.shape,index,(1,2));speckled=time.perf_counter()
            optical=camera.render(objects);photographed=time.perf_counter()
            sc.to_decibels(noisy);finished=time.perf_counter()
            if index>=2:
                for key,value in zip(timing,((rendered-start)*1000,(speckled-rendered)*1000,(photographed-speckled)*1000,(finished-photographed)*1000)):timing[key].append(value)
        summary={key:dict(mean_ms=float(np.mean(v)),std_ms=float(np.std(v,ddof=1)),samples=20) for key,v in timing.items()}
        records.append(dict(target=target,triangles=triangles,analytic_planes=1,config=cfg,stages=summary,
                            render_only_frames_per_second=1000/summary['Cython_scene_and_CPP_render_ms']['mean_ms']))
report=dict(platform=platform.platform(),python=sys.version,extension=sonar.__file__,records=records,
            caveat='Render throughput excludes optical, speckle and GUI. Cython time includes scene conversion and OBJ loading. Ray visibility and deposition share a loop and are not timed separately.')
with open(outputs.output_path('benchmark.json'),'w') as f:json.dump(report,f,indent=2)
print(json.dumps(report,indent=2))
