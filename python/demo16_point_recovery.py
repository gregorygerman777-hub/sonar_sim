import sys,json
sys.path[:0]=['.','python']
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import point_recovery as pr,outputs

truth=np.array([.3,4.,.4]);trials=300;rng=np.random.default_rng(20260908)
scenarios={'sideways two views':[(np.array([x,0.,0.]),np.eye(3)) for x in (-.3,.3)],
           'vertical two views':[(np.array([0.,0.,z]),np.eye(3)) for z in (-.3,.3)],
           'mixed eight views':[(np.array([.3*np.cos(a),0,.3*np.sin(a)]),np.eye(3)) for a in np.linspace(0,2*np.pi,8,endpoint=False)]}
records={};fig,axs=plt.subplots(1,2,figsize=(12,4),constrained_layout=True)
for name,poses in scenarios.items():
    scale=np.tile([.003,np.radians(.05)],len(poses));s=np.linalg.svd(pr.jacobian(truth,poses)/scale[:,None],compute_uv=False)
    singular_flat=np.linalg.svd(pr.jacobian([.3,4,0],poses)/scale[:,None],compute_uv=False)
    exact=pr.solve(pr.observe(truth,poses),poses,[0,3.8,.2]);errors=[]
    for _ in range(trials):
        # Position and small yaw/tilt/roll errors perturb the measurement poses;
        # reconstruction receives the nominal, imperfect poses.
        actual=[]
        for pos,axes in poses:
            angles=rng.normal(0,np.radians(.05),3);x,y,z=angles
            rx=np.array([[1,0,0],[0,np.cos(x),-np.sin(x)],[0,np.sin(x),np.cos(x)]])
            ry=np.array([[np.cos(y),0,np.sin(y)],[0,1,0],[-np.sin(y),0,np.cos(y)]])
            rz=np.array([[np.cos(z),-np.sin(z),0],[np.sin(z),np.cos(z),0],[0,0,1]])
            actual.append((pos+rng.normal(0,.002,3),axes@rx@ry@rz))
        measured=pr.observe(truth,actual)+rng.normal(size=len(scale))*scale
        recovered=pr.solve(measured,poses,[0,3.8,.2]);errors.append(np.linalg.norm(recovered-truth))
    errors=np.array(errors);ci=np.quantile(np.mean(rng.choice(errors,(2000,trials)),axis=1),[.025,.975])
    records[name]=dict(noise_free_error_m=float(np.linalg.norm(exact-truth)),singular_values=s.tolist(),rank=int(np.sum(s>s[0]*1e-8)),
        zero_elevation_singular_values=singular_flat.tolist(),mean_error_m=float(errors.mean()),std_m=float(errors.std(ddof=1)),
        ci95_mean_m=ci.tolist(),failure_rate_over_10cm=float(np.mean(errors>.1)),trials=trials)
    axs[0].hist(errors*100,bins=30,alpha=.5,label=name);axs[1].semilogy(range(1,4),s,'o-',label=name)
axs[0].set(xlabel='3-D point error (cm)',ylabel='trials');axs[1].set(xlabel='sensitivity direction',ylabel='noise-scaled singular value')
for ax in axs:ax.legend(fontsize=8);ax.grid(alpha=.2)
fig.savefig(outputs.output_path('demo16_point_recovery.png'),dpi=140)
records['assumptions']='Known point correspondence; positive-elevation initialization selects one branch. Coplanar sideways poses retain a global +/- elevation ambiguity even when local rank is 3. Position noise 2 mm; attitude noise 0.05 deg; range noise 3 mm; bearing noise 0.05 deg.'
with open(outputs.output_path('demo16_metrics.json'),'w') as f:json.dump(records,f,indent=2)
print(json.dumps(records,indent=2))
