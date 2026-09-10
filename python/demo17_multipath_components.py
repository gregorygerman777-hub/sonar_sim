"""Separate direct, ghost, mirror, diagnostic mask and roughness response."""
import sys,json
sys.path[:0]=['.','python']
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sonar,scene as sc,outputs

objects=[sonar.make_sphere((0,4,-.4),.35,.8)];position=(0,0,-.6)
config=dict(num_azimuth_bins=97,num_range_bins=350,num_elevation_subrays=512,
            frequency_hz=1.8e6,max_range_m=7,vertical_beamwidth_deg=14,horizontal_fov_deg=45,
            multipath_enabled=True,num_threads=1,beam_mode='top_hat')
fig,axs=plt.subplots(3,5,figsize=(17,9),constrained_layout=True);records=[]
for row,roll in enumerate((0,45,90)):
    axes=sc.tilted_axes(0,roll);components=[]
    for key in ('direct_enabled','ghost_enabled','mirror_enabled'):
        flags=dict(direct_enabled=False,ghost_enabled=False,mirror_enabled=False);flags[key]=True
        sim=sonar.SonarSimulator(**config,**flags);components.append(sim.render(objects,position,axes))
    combined=sonar.SonarSimulator(**config).render(objects,position,axes)
    residual=float(np.max(np.abs(sum(components)-combined)));peak=combined.max()
    for col,(im,name) in enumerate(zip(components+[combined],('direct','ghost','mirror','combined'))):
        sc.show_image(axs[row,col],im,sim,reference=peak,cmap='Greens');axs[row,col].set(title=f'{roll} deg roll / {name}',xlabel='range m',ylabel='bearing deg',xlim=(3,5))
    codes=np.zeros(combined.shape,dtype=np.uint8)
    for index,im in enumerate(components):codes[im>peak*.001]=index+1
    colours=np.array([[5,14,20],[67,244,143],[245,104,163],[75,177,255]],dtype=np.uint8)
    axs[row,4].imshow(colours[codes],origin='lower',aspect='auto');axs[row,4].set(title='green direct / pink ghost / blue mirror',xlabel='range bin',ylabel='bearing bin')
    records.append(dict(roll_deg=roll,energy=[float(im.sum()) for im in components],component_sum_error=residual))
fig.savefig(outputs.output_path('demo17_components.png'),dpi=130)

roughness=np.linspace(0,.5,41);curve=[]
for mm in roughness:
    im=sonar.SonarSimulator(**config,direct_enabled=False,mirror_enabled=False,surface_rms_height_m=mm/1000).render(objects,position,sc.tilted_axes(0,90))
    curve.append(float(im.sum()))
curve=np.array(curve)/curve[0]
fig,ax=plt.subplots(figsize=(7,4),constrained_layout=True);ax.plot(roughness,curve,'o-',ms=3)
ax.set(xlabel='RMS surface height (mm)',ylabel='coherent ghost energy / calm energy',title='1.8 MHz: measured rendered-patch roughness sweep');ax.grid(alpha=.2)
fig.savefig(outputs.output_path('demo17_roughness.png'),dpi=140)
report=dict(config=config,roll=records,roughness_mm=roughness.tolist(),relative_ghost_energy=curve.tolist(),
            approximation='Direct-visible patches seed all components; reflected-leg occlusion is not evaluated; no ghost removal.')
with open(outputs.output_path('demo17_metrics.json'),'w') as f:json.dump(report,f,indent=2)
print(json.dumps(dict(roll=records,ghost_at_half_mm=float(curve[-1])),indent=2))
assert records[0]['energy'][1]>0 and records[0]['energy'][2]==0, 'ghost should survive when mirror leaves vertical FOV'
