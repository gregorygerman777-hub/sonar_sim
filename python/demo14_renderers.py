"""Four renderer modes, OBJ projection, convergence and array response diagnostics."""
import sys,time,json
sys.path[:0]=['.','python']
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sonar,scene as sc,outputs,mesh_assets,speckle

assets=mesh_assets.generate()
position=np.array([0.,0.,-0.6]);axes=sc.tilted_axes(14,35)
objects=[sonar.make_plane((0,0,-2),(0,0,1),0.08),sonar.make_sphere((0,4,-1.3),0.35,0.8)]
config=dict(frequency_hz=900e3,num_azimuth_bins=97,num_range_bins=400,max_range_m=9,
            vertical_beamwidth_deg=30,num_elevation_subrays=512,num_threads=1)
images=[];records=[]
for label,options in [('one ray',dict(num_elevation_subrays=1)),('integrated direct',{}),
                      ('surface multipath',dict(multipath_enabled=True)),
                      ('roughness + correlated speckle',dict(multipath_enabled=True,surface_rms_height_m=0.0001))]:
    sim=sonar.SonarSimulator(**dict(config,**options));start=time.perf_counter()
    image=sim.render(objects,position,axes)
    elapsed=1000*(time.perf_counter()-start)
    if len(images)==3:image*=speckle.factors(image.shape,42,(1,2))
    images.append(image);records.append(dict(mode=label,milliseconds=elapsed,total=float(image.sum()),peak=float(image.max())))
fig,axs=plt.subplots(2,4,figsize=(16,8),constrained_layout=True)
peak=max(im.max() for im in images)
for i,(im,record) in enumerate(zip(images,records)):
    sc.show_image(axs[0,i],im,sim,reference=peak,cmap='Greens');axs[0,i].set(title=record['mode'],xlabel='range (m)',ylabel='bearing (deg)')
    axs[1,i].imshow((im-images[1]).T,aspect='auto',cmap='RdBu_r',vmin=-peak,vmax=peak)
    axs[1,i].set(title=f"difference from direct; {record['milliseconds']:.1f} ms",xlabel='bearing bin',ylabel='range bin')
fig.savefig(outputs.output_path('demo14_renderers.png'),dpi=130)

counts=[32,64,128,256,512,1024,2048,4096]
reference=sonar.SonarSimulator(**dict(config,num_elevation_subrays=8192)).render(objects,position,axes)
errors=[]
for n in counts:
    im=sonar.SonarSimulator(**dict(config,num_elevation_subrays=n)).render(objects,position,axes)
    errors.append(float(np.abs(im-reference).sum()/reference.sum()))
fig,ax=plt.subplots(figsize=(7,4),constrained_layout=True);ax.loglog(counts,errors,'o-')
ax.set(xlabel='elevation sub-rays',ylabel='relative L1 image error versus 8192 rays',title='Shadow scene convergence, normalized angular average');ax.grid(alpha=.2)
fig.savefig(outputs.output_path('demo14_convergence.png'),dpi=140)

angle=np.linspace(-15,15,12001);patterns={};beam_metrics={}
for mode in ('array','hann'):
    p=np.array([sonar.beam_response(np.radians(a),mode=mode) for a in angle]);patterns[mode]=p
    centre=len(p)//2;right=p[centre:];minima=np.where((right[1:-1]<right[:-2])&(right[1:-1]<right[2:]))[0]+1
    first=int(minima[0]);half=np.where(right<.5)[0][0]
    beam_metrics[mode]=dict(half_power_width_deg=float(2*angle[centre+half]),first_null_deg=float(angle[centre+first]),
                          sidelobe_db=float(10*np.log10(right[first:].max())))
fig,ax=plt.subplots(figsize=(7,4),constrained_layout=True)
for mode,p in patterns.items():ax.plot(angle,10*np.log10(np.maximum(p,1e-8)),label=mode)
ax.set(ylim=(-70,1),xlabel='elevation offset (degrees)',ylabel='beam power (dB)',title='Ideal array: half-wavelength spacing, 64 elements');ax.legend()
fig.savefig(outputs.output_path('demo14_beams.png'),dpi=140)

fig=plt.figure(figsize=(15,6),constrained_layout=True)
meshstats=[]
for i,name in enumerate(('box','coral_rock','industrial_pipe','concave_table')):
    vertices,faces=assets[name];vertices=vertices+np.array([0,4,0])
    sim=sonar.SonarSimulator(num_azimuth_bins=65,num_range_bins=200,num_elevation_subrays=128,
                             vertical_beamwidth_deg=30,beam_mode='top_hat',num_threads=1,max_range_m=7)
    start=time.perf_counter();im=sim.render([sonar.make_mesh('assets/'+name+'.obj',translation=(0,4,0))]);ms=1000*(time.perf_counter()-start)
    ax=fig.add_subplot(2,4,i+1,projection='3d')
    for f in faces: p=vertices[np.r_[f,f[0]]];ax.plot(*p.T,color='#167c83',linewidth=.4)
    ax.set(title=name,xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)');ax.set_box_aspect((1,1,1))
    ax=fig.add_subplot(2,4,i+5);sc.show_image(ax,im,sim,cmap='Greens')
    # Projection is explicit range and bearing; elevation is collapsed.
    r=np.linalg.norm(vertices,axis=1);bearing=np.degrees(np.arctan2(vertices[:,0],vertices[:,1]))
    for f in faces:
        ids=np.r_[f,f[0]];ax.plot(r[ids],bearing[ids],color='orange',linewidth=.3,alpha=.3)
    ax.set(xlabel='range (m)',ylabel='bearing (deg)',title=f'{len(faces)} triangles; {ms:.1f} ms')
    meshstats.append(dict(name=name,triangles=len(faces),milliseconds=ms))
fig.savefig(outputs.output_path('demo14_meshes.png'),dpi=130)
report=dict(config=config,renderer=records,convergence=dict(counts=counts,relative_L1=errors),beam=beam_metrics,meshes=meshstats)
with open(outputs.output_path('demo14_metrics.json'),'w') as f:json.dump(report,f,indent=2)
print(json.dumps(report,indent=2))
