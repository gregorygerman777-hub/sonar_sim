"""Feasible-volume reconstruction and paired Monte Carlo degradation curves."""
import sys,json
sys.path[:0]=['.','python']
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sonar,reconstruction as rc,speckle,outputs

STEP=.08;CENTRE=np.array([0.,4.,0.]);RADIUS=.45
points,shape=rc.grid(CENTRE,step=STEP)
truth=np.linalg.norm(points-CENTRE,axis=1)<=RADIUS
sim=sonar.SonarSimulator(num_azimuth_bins=65,num_range_bins=220,num_elevation_subrays=257,
                         vertical_beamwidth_deg=30,beam_mode='top_hat',max_range_m=7,num_threads=1)
objects=[sonar.make_sphere(CENTRE,RADIUS,.8)]
poses=list(rc.orbit_poses(32));images=[sim.render(objects,p,a) for p,a in poses]
records=[];volumes=[]
for count in (4,8,16,32):
    stride=32//count;kept=rc.reconstruct(images[::stride],poses[::stride],sim,points)
    records.append(dict(poses=count,**rc.metrics(kept,truth,STEP)));volumes.append(kept)
fig=plt.figure(figsize=(15,7),constrained_layout=True)
for i,(kept,record) in enumerate(zip(volumes,records)):
    ax=fig.add_subplot(2,4,i+1,projection='3d');p=points[kept]
    ax.scatter(*p.T,s=2,c='#137f85',alpha=.4);ax.set(title=f"{record['poses']} poses; IoU {record['iou']:.3f}",xlabel='X',ylabel='Y',zlabel='Z');ax.set_box_aspect((1,1,1))
    ax=fig.add_subplot(2,4,i+5)
    codes=(truth.astype(int)+2*kept.astype(int)).reshape(shape)
    ax.imshow(codes[:,shape[1]//2,:].T,origin='lower',vmin=0,vmax=3,cmap='viridis')
    ax.set(title='slice: 0 empty, 1 missed, 2 excess, 3 overlap',xlabel='X voxel',ylabel='Z voxel')
fig.savefig(outputs.output_path('demo15_carving.png'),dpi=140)
# Exposed voxel faces form a block surface; no smoothing or ground-truth fitting.
v,f=rc.surface_mesh(volumes[-1],points,shape,STEP)
with open(outputs.output_path('demo15_voxel_surface.obj'),'w') as out:
    for p in v:out.write('v '+' '.join(map(str,p))+'\n')
    for p in np.array(f)+1:out.write('f '+' '.join(map(str,p))+'\n')
np.savez_compressed(outputs.output_path('demo15_volume.npz'),points=points,truth=truth,initial=np.ones(len(points),bool),surviving=volumes[-1],carved=~volumes[-1],voxel_size=STEP)

TRIALS=20
sweeps={'looks':[1,2,4,8],'contrast':[0,.3,.6,1], 'correlation':[0,1,2,4],
        'threshold':[.005,.02,.05,.15],'snr_db':[5,10,20,40]}
curves={};base_images=images[::4];base_poses=poses[::4]
fig,axs=plt.subplots(1,5,figsize=(18,4),constrained_layout=True)
for ax,(parameter,values) in zip(axs,sweeps.items()):
    rows=[]
    for value in values:
        scores=[]
        for trial in range(TRIALS):
            noisy=[]
            for j,im in enumerate(base_images):
                kw=dict(looks=1,contrast=1.,correlation=(0,0))
                if parameter=='correlation':kw['correlation']=(value,value)
                if parameter in ('looks','contrast'):kw[parameter]=value
                observed=im*speckle.factors(im.shape,10000+trial*32+j,**kw)
                if parameter=='snr_db':
                    # Power SNR relative to peak true intensity; exponential receiver power floor.
                    observed+=np.random.default_rng(20000+trial*32+j).exponential(im.max()*10**(-value/10),im.shape)
                noisy.append(observed)
            kept=rc.reconstruct(noisy,base_poses,sim,points,threshold=value if parameter=='threshold' else .03)
            scores.append(rc.metrics(kept,truth,STEP)['iou'])
        rng=np.random.default_rng(77);means=np.mean(rng.choice(scores,(2000,TRIALS)),axis=1)
        rows.append(dict(value=value,mean=float(np.mean(scores)),ci95=np.quantile(means,[.025,.975]).tolist(),trials=TRIALS,scores=scores))
    curves[parameter]=rows
    mean=np.array([r['mean'] for r in rows]);bounds=np.array([r['ci95'] for r in rows])
    ax.errorbar(values,mean,yerr=np.maximum(0,np.array([mean-bounds[:,0],bounds[:,1]-mean])),fmt='o-')
    ax.set(xlabel=parameter,ylabel='volume IoU',title='20 trials; 95% bootstrap mean CI',ylim=(0,1));ax.grid(alpha=.2)
fig.savefig(outputs.output_path('demo15_degradation.png'),dpi=140)
report=dict(voxel_size_m=STEP,grid_shape=shape,voxel_count=len(points),sphere_radius_m=RADIUS,poses=records,sweeps=curves,
            limitation='Conservative farther-range feasible shadow from segmented highlights; synthetic sphere and known poses only; CIs are across random pings, not real environments.')
with open(outputs.output_path('demo15_metrics.json'),'w') as f:json.dump(report,f,indent=2)
print(json.dumps(dict(poses=records,mean_IoU={k:[r['mean'] for r in v] for k,v in curves.items()}),indent=2))
