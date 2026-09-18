"""Dense, leakage-separated external evaluation of sonar_sim's SLAM front end."""
import sys,json,hashlib,time,math,argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'python')]
import numpy as np
import pandas as pd
from PIL import Image
from scipy.ndimage import maximum_filter
import sonar,slam
from benchmark_public_uxo import ArisAxes,reference,score,rotate_xy

def features_fast(image,axes):
    # Exact ordering/tie handling of slam.extract_features, accelerated candidates.
    v=np.asarray(image,float)
    mask=(v>=.025*v.max())&(v>=maximum_filter(v,size=3,mode='constant',cval=-np.inf))
    mask[[0,-1],:]=False;mask[:,[0,-1]]=False
    b,r=np.where(mask); order=np.lexsort((-r,-b,-v[b,r]))
    selected=[]
    for k in order:
        if any(abs(int(b[k])-ob)<=2 and abs(int(r[k])-orr)<=4 for ob,orr in selected):continue
        selected.append((int(b[k]),int(r[k])))
        if len(selected)==100:break
    if not selected:return np.empty((0,2))
    b,r=np.array(selected).T;t=np.radians(axes.angles[b]);d=axes.ranges[r]
    return np.column_stack((d*np.sin(t),d*np.cos(t)))

def load_inputs(recording,angles,cache):
    # Explicit whitelist is the only metadata visible to estimation.
    cols=['FrameIndex','FrameTime','SamplesPerBeam','SampleStartDelay','SamplePeriod','SoundSpeed']
    meta=pd.read_csv(recording/'aris_frame_meta.csv',usecols=cols).set_index('FrameIndex')
    files=sorted((recording/'aris_raw').glob('*.pgm'));ids=np.array([int(p.stem) for p in files])
    assert len(files)==len(meta) and set(ids)==set(meta.index)
    times=meta.loc[ids,'FrameTime'].to_numpy(float)*1e-6;times-=times[0]
    assert np.all(np.diff(times)>0)
    digest=hashlib.sha256(Path(__file__).read_bytes())
    digest.update(json.dumps(angles,sort_keys=True).encode())
    digest.update(meta.to_csv().encode())
    hashes={}
    for p in files:
        h=hashlib.sha256(p.read_bytes()).hexdigest();hashes[p.name]=h;digest.update(h.encode())
    key=digest.hexdigest();cp=cache/(key+'.npz')
    if cp.exists():
        a=np.load(cp);points=list(a['points']);desc=list(a['descriptors']);lengths=a['lengths']
        points=[p[:n] for p,n in zip(points,lengths)]
    else:
        points=[];desc=[]
        for i,p in enumerate(files):
            raw=np.asarray(Image.open(p),float);axis=ArisAxes(meta.loc[int(p.stem)],angles[raw.shape[1]])
            assert raw.shape[0]==len(axis.ranges)
            points.append(features_fast(raw.T,axis));desc.append(slam.scan_descriptor(raw.T))
        padded=np.zeros((len(points),100,2));lengths=[]
        for i,p in enumerate(points):padded[i,:len(p)]=p;lengths.append(len(p))
        np.savez_compressed(cp,points=padded,lengths=lengths,descriptors=desc)
    return dict(ids=ids,times=times,points=points,desc=desc,hashes=hashes,cache_key=key)

def estimate(inputs,stride):
    begin=time.perf_counter()
    ix=np.unique(np.r_[np.arange(0,len(inputs['ids']),stride),len(inputs['ids'])-1]).astype(int)
    t=inputs['times'][ix];pts=[inputs['points'][i] for i in ix]
    poses=[np.zeros(3)];motion=np.zeros(3);edges=[]
    for i in range(1,len(ix)):
        initial=motion.copy()
        if i>1:initial*=(t[i]-t[i-1])/(t[i-1]-t[i-2])
        m,e,n=slam.icp_relative(pts[i-1],pts[i],initial=initial,max_correspondence_m=.60)
        ok=n>=5 and e<.22;motion=np.array(m) if ok else np.zeros(3)
        old=poses[-1];xy=slam.transform_points(motion[None,:2],old)[0]
        poses.append(np.r_[xy,old[2]+motion[2]])
        edges.append(dict(accepted=bool(ok),inliers=int(n),residual=float(e) if np.isfinite(e) else None))
    poses=np.array(poses)
    keys=np.unique(np.linspace(0,len(ix)-1,min(81,len(ix))).round().astype(int))
    graph=sonar.PlanarSlam(initial=(0,0,0))
    for a,b in zip(keys[:-1],keys[1:]):
        m=sonar.relative_pose_2d(poses[a],poses[b]); es=edges[a:b]
        # Root-sum-square accumulated pair residual as a documented heuristic.
        sigma=np.sqrt(sum(max(.025,x['residual'])**2 if x['accepted'] else 100 for x in es))
        graph.add_odometry(m,sigma_translation=float(sigma),sigma_yaw=math.radians(.8)*np.sqrt(b-a))
    desc=[inputs['desc'][ix[k]] for k in keys];loops=[]
    for i in range(12,len(keys)):
        j,sim=slam.candidate_loop(desc,i,minimum_separation=12)
        if j is None or sim<.91:continue
        initial=sonar.relative_pose_2d(poses[keys[j]],poses[keys[i]])
        if np.linalg.norm(initial[:2])>1.5:continue
        m,e,n=slam.icp_relative(pts[keys[j]],pts[keys[i]],initial=initial,max_correspondence_m=max(.7,np.linalg.norm(initial[:2])+.35))
        if n>=6 and e<.18:
            graph.add_loop_closure(j,i,m,sigma_translation=max(.02,e),sigma_yaw=math.radians(.6))
            loops.append(dict(first=int(j),second=int(i),measurement=np.asarray(m).tolist(),residual=float(e),inliers=int(n),similarity=float(sim)))
    if loops:graph.optimize(iterations=40,huber_delta=2.5)
    return dict(ix=ix,times=t,poses=poses,keys=keys,optimized=graph.poses().copy(),edges=edges,loops=loops,seconds=time.perf_counter()-begin)

def rmse(x):return float(np.sqrt(np.mean(np.square(x))))
def wrap(x):return np.arctan2(np.sin(x),np.cos(x))
def evaluate(recording,inputs,result,stride,out,split):
    # All reference access occurs only after estimation.
    ix=result['ix'];truth,xyz,gimbal=reference(recording,inputs['ids'][ix]);keys=result['keys']
    tk=truth[keys];before=result['poses'][keys];after=result['optimized']
    aligned,ae=score(after[:,:2],tk[:,:2]);_,be=score(before[:,:2],tk[:,:2])
    stationary=rmse(np.linalg.norm(tk[:,:2]-tk[:,:2].mean(0),axis=1))
    rpe=[];yrpe=[];lags=[]
    t=result['times'];p=result['poses']
    for i in range(len(p)):
        j=np.searchsorted(t,t[i]+1.)
        if j>=len(p):continue
        candidates=[j]+([j-1] if j-1>i else [])
        j=min(candidates,key=lambda j:abs(t[j]-t[i]-1))
        if abs(t[j]-t[i]-1)>.15:continue
        et=np.asarray(sonar.relative_pose_2d(p[i],p[j]));gt=np.asarray(sonar.relative_pose_2d(truth[i],truth[j]))
        rpe.append(np.linalg.norm(et[:2]-gt[:2]));yrpe.append(np.degrees(wrap(et[2]-gt[2])));lags.append(t[j]-t[i])
    for loop in result['loops']:
        gt=np.asarray(sonar.relative_pose_2d(tk[loop['first']],tk[loop['second']]))
        d=np.array(loop['measurement'])-gt
        loop.update(reference_translation_error_m=float(np.linalg.norm(d[:2])),reference_yaw_error_deg=float(abs(np.degrees(wrap(d[2])))))
    initial=rotate_xy(after[:,:2],tk[0,2])+tk[0,:2]
    metrics=dict(sequence=recording.name,split=split,stride=stride,raw_frames=len(inputs['ids']),processed_frames=len(p),evaluation_keyframes=len(keys),
      odometry_ate_m=rmse(be),loop_ate_m=rmse(ae),stationary_ate_m=stationary,loop_change_m=rmse(ae)-rmse(be),
      initial_alignment_ate_m=rmse(np.linalg.norm(initial-tk[:,:2],axis=1)),
      rpe_1s_translation_m=rmse(rpe) if rpe else None,rpe_1s_yaw_deg=rmse(yrpe) if yrpe else None,rpe_pairs=len(rpe),
      rpe_actual_lag_minmax_s=[float(min(lags)),float(max(lags))] if lags else None,
      estimated_length_m=float(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1).sum()),reference_length_m=float(np.linalg.norm(np.diff(truth[:,:2],axis=0),axis=1).sum()),
      rejected_pairs=sum(not e['accepted'] for e in result['edges']),loop_constraints=len(result['loops']),
      median_loop_translation_error_m=float(np.median([l['reference_translation_error_m'] for l in result['loops']])) if result['loops'] else None,
      median_loop_yaw_error_deg=float(np.median([l['reference_yaw_error_deg'] for l in result['loops']])) if result['loops'] else None,
      duration_s=float(t[-1]),dt_quantiles_s=np.quantile(np.diff(t),[0,.5,1]).tolist(),tilt_minmax_deg=[float(gimbal[:,1].min()),float(gimbal[:,1].max())],vertical_span_m=float(np.ptp(xyz[:,2])),estimate_seconds=result['seconds'])
    folder=out/f'{recording.name}_stride{stride}';folder.mkdir()
    pd.DataFrame(dict(frame_id=inputs['ids'][ix],time_s=t,x=p[:,0],y=p[:,1],yaw=p[:,2],gt_x=truth[:,0],gt_y=truth[:,1],gt_yaw=truth[:,2])).to_csv(folder/'odometry.csv',index=False)
    pd.DataFrame(dict(frame_id=inputs['ids'][ix[keys]],time_s=t[keys],before_x=before[:,0],before_y=before[:,1],after_x=after[:,0],after_y=after[:,1],after_yaw=after[:,2],gt_x=tk[:,0],gt_y=tk[:,1],gt_yaw=tk[:,2],aligned_x=aligned[:,0],aligned_y=aligned[:,1])).to_csv(folder/'keyframes.csv',index=False)
    (folder/'metrics.json').write_text(json.dumps(metrics,indent=2));(folder/'constraints.json').write_text(json.dumps(dict(edges=result['edges'],loops=result['loops']),indent=2))
    return metrics

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--data',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--split',choices=['development','heldout'],required=True);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    angles={int(k):v for k,v in json.loads((ROOT/'data_external/dfki_uxo_2024/beam_angles.json').read_text()).items()}
    cache=ROOT/'data_external/dfki_uxo_2024/feature_cache';cache.mkdir(exist_ok=True)
    metrics=[]
    for rec in sorted(p.parent for p in args.data.rglob('gantry.csv')):
        start=time.perf_counter();inputs=load_inputs(rec,angles,cache)
        print(rec.name,'loaded',len(inputs['ids']),'frames',round(time.perf_counter()-start,1),'s',flush=True)
        (args.output/(rec.name+'_inputs.json')).write_text(json.dumps(dict(raw_sha256=inputs['hashes'],cache_key=inputs['cache_key']),indent=2))
        # Freeze all estimates across rates before this sequence's evaluation.
        results=[(s,estimate(inputs,s)) for s in [1,5,20]]
        for s,result in results:
            m=evaluate(rec,inputs,result,s,args.output,args.split);metrics.append(m);print(json.dumps(m),flush=True)
        (args.output/'summary.json').write_text(json.dumps(metrics,indent=2))
    (args.output/'evaluate_snapshot.py').write_text(Path(__file__).read_text())
if __name__=='__main__':main()
