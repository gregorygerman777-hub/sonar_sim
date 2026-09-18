"""Generate descriptive, sequence-level paired statistics and figures."""
from evaluate import *
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent

def main():
 data=[]
 for split in ['development','heldout']:
  data+=json.loads((HERE/'runs'/split/'summary.json').read_text())
 df=pd.DataFrame(data);df.to_csv(HERE/'all_metrics.csv',index=False)
 rng=np.random.default_rng(20260914);cis=[]
 for split in ['development','heldout']:
  d=df[(df.split==split)&(df.stride==1)]
  for name,values in [('loop_minus_odometry',d.loop_ate_m-d.odometry_ate_m),('odometry_minus_stationary',d.odometry_ate_m-d.stationary_ate_m)]:
   v=values.to_numpy();boot=rng.choice(v,(10000,len(v)),replace=True).mean(1)
   cis.append(dict(split=split,contrast=name,n_sequences=len(v),mean_difference_m=float(v.mean()),descriptive_percentile95_m=np.quantile(boot,[.025,.975]).tolist()))
 (HERE/'sequence_bootstrap.json').write_text(json.dumps(cis,indent=2))
 # Reference synchronization perturbation is scoring-only; never feeds estimator.
 sensitivity=[]
 for _,row in df[df.stride==1].iterrows():
  root=ROOT/'data_external/dfki_uxo_2024'/('uxo_samples' if row.split=='development' else 'holdout/recordings')
  rec=next(p.parent for p in root.rglob('gantry.csv') if p.parent.name==row.sequence)
  m=pd.read_csv(rec/'aris_frame_meta.csv');ids=m.FrameIndex.to_numpy();tt=m.FrameTime.to_numpy(float)*1e-6;tt-=tt[0]
  truth,_,_=reference(rec,ids)
  k=pd.read_csv(HERE/'runs'/row.split/f'{row.sequence}_stride1'/'keyframes.csv')
  keep=(k.time_s>=.1)&(k.time_s<=tt[-1]-.1);k=k[keep]
  vals={}
  for shift in [-.1,0,.1]:
   xy=np.column_stack([np.interp(k.time_s+shift,tt,truth[:,c]) for c in [0,1]])
   _,e=score(k[['after_x','after_y']].to_numpy(),xy);vals[str(shift)]=rmse(e)
  sensitivity.append(dict(sequence=row.sequence,shift_ate_m=vals,max_change_from_zero_m=max(abs(v-vals['0']) for v in vals.values())))
 (HERE/'time_shift_sensitivity.json').write_text(json.dumps(sensitivity,indent=2))
 fig,axs=plt.subplots(2,4,figsize=(16,8),constrained_layout=True)
 for ax,(_,row) in zip(axs.ravel(),df[df.stride==1].iterrows()):
  k=pd.read_csv(HERE/'runs'/row.split/f'{row.sequence}_stride1'/'keyframes.csv')
  truth=k[['gt_x','gt_y']].to_numpy();origin=truth[0]
  before,_=score(k[['before_x','before_y']].to_numpy(),truth)
  ax.plot(*(truth-origin).T,'k-',lw=2,label='Reference')
  ax.plot(*(before-origin).T,color='#d26a28',ls='--',label='Odometry')
  ax.plot(*(k[['aligned_x','aligned_y']].to_numpy()-origin).T,color='#087e8b',label='With loops')
  ax.set_title(row.sequence+'\n'+row.split,fontsize=10);ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2);ax.set(xlabel='X (m)',ylabel='Y (m)')
 axs[0,0].legend(fontsize=8);fig.suptitle('All-frame sonar SLAM: proper SE(2) alignment, no fitted scale\nTilted tank data; planar-model mismatch remains')
 fig.savefig(HERE/'trajectories.png',dpi=150);plt.close(fig)
 fig,axs=plt.subplots(1,2,figsize=(12,4.5),constrained_layout=True)
 for ax,split in zip(axs,['development','heldout']):
  for label,column,color in [('Odometry','odometry_ate_m','#d26a28'),('With loops','loop_ate_m','#087e8b'),('Stationary','stationary_ate_m','#333333')]:
   d=df[df.split==split].groupby('stride')[column].mean();ax.plot(d.index,d,'o-',label=label,color=color)
  ax.set(xlabel='Process every Nth raw frame',ylabel='Mean sequence ATE (m)',title=split);ax.set_xticks([1,5,20]);ax.grid(alpha=.2);ax.legend()
 fig.suptitle('Temporal-spacing sensitivity (keyframe timestamps differ across rates)');fig.savefig(HERE/'spacing.png',dpi=150);plt.close(fig)
 lines=['# External sonar SLAM evaluation','',
 '## Conclusion','',
 'The current adapter does **not yet validate** sonar SLAM on these real ARIS recordings. Dense results are inconsistent, and physical bearing handedness remains unresolved. The original frozen results are retained as conditional baseline measurements; they must not be presented as definitive algorithm accuracy or simulator-physics validation.',
 '',f'Processed {int(df[df.stride==1].raw_frames.sum()):,} distinct real sonar frames in eight complete sequences. Five recordings are development data; three were selected before their imagery or errors were read. Each ran at three frame spacings, with paired before/after loop-closure scores: 24 configurations. The three new recordings are the same object and site, so they do not establish cross-site generalization.','',
 '## All-frame result','',
 '| Split / recording | Odometry ATE (m) | With loops (m) | Stationary (m) | Loops | 1 s translation RPE (m) |',
 '|---|---:|---:|---:|---:|---:|']
 for _,r in df[df.stride==1].iterrows():lines.append(f'| {r.split} / {r.sequence} | {r.odometry_ate_m:.3f} | {r.loop_ate_m:.3f} | {r.stationary_ate_m:.3f} | {r.loop_constraints} | {r.rpe_1s_translation_m:.3f} |')
 for split in ['development','heldout']:
  d=df[(df.split==split)&(df.stride==1)]
  lines += ['',f'{split}: odometry beats stationary on {int((d.odometry_ate_m<d.stationary_ate_m).sum())}/{len(d)} sequences; with loops beats stationary on {int((d.loop_ate_m<d.stationary_ate_m).sum())}/{len(d)}. Loops worsen ATE on {int((d.loop_change_m>1e-6).sum())}/{int((d.loop_constraints>0).sum())} runs that accepted loops.']
 lines+=['','![Trajectories](trajectories.png)','','## What makes this a stronger test','',
 '- Complete raw sequences, fixed settings, real published reference motion, and explicitly separated development/test recordings.',
 '- Odometry versus loop closure; three temporal spacings; fixed-time translation and yaw RPE; stationary controls; explicit failures and constraints.',
 '- Known-transform recovery, pose composition against C++, accelerated-loader equivalence including tied maxima, and a reference-field poisoning test.',
 '- Input hashes, member CRC validation for all new files, source snapshots, and independent homogeneous calibration composition.',
 '- Sequence-level uncertainty summaries rather than treating thousands of correlated frames as independent trials. The estimator is deterministic; repeating identical runs is not a new experiment.',
 '', '## Geometry and timing audit','',
 'The range bins use round-trip travel time and measured sound speed. Nonuniform ARIS beam centers are retained. The adapter follows the publisher’s final polar2 display mapping, x = r sin(bearing), forward = r cos(bearing). However, the raw viewer explicitly describes beams as right-to-left and flips them. Display orientation alone does not certify physical x-right handedness. A known off-axis target calibration is still required to settle that convention. The reference chain includes the fixed mounting quaternion, gimbal xyz Euler rotation, and rotated -0.108725 m lever arm. Independent homogeneous composition agrees numerically with the scorer. This checks implementation against the published model, not the hardware calibration itself.',
 '', 'Tilts of 22–60 degrees violate a horizontal-plane interpretation. For a zero-elevation central ray, horizontal forward displacement is r cos(tilt), i.e. approximately 0.93r to 0.50r. This is a geometry diagnostic, not a valid blanket correction for unknown 3-D scene elevation. ATE and horizontal RPE therefore describe an intentionally mismatched planar baseline.',
 '', f'Shifting scoring reference timestamps by ±0.1 s, excluding boundary frames, changed all-frame post-loop ATE by at most {max(x["max_change_from_zero_m"] for x in sensitivity):.4f} m. This is a local sensitivity analysis, not a measured synchronization uncertainty or validation.',
 '', '## Temporal spacing','', '![Temporal-spacing sensitivity](spacing.png)',
 '', 'Dense processing can accumulate registration bias and jitter. Accepted matches do not imply correct motion. Loop proposals in this object-centric dataset can repeatedly match the same object from a different viewpoint, rather than establish a true revisit. Keyframe selection and the legacy 12-keyframe separation vary with stride, so the spacing experiment changes the complete pipeline; it does not isolate only ICP frame spacing.',
 '', '## Descriptive uncertainty','', '| Split | Paired contrast (positive is worse) | Mean difference (m) | Sequence bootstrap 95% interval (m) |', '|---|---|---:|---|']
 for c in cis:
  a,b=c['descriptive_percentile95_m'];lines.append(f'| {c["split"]} | {c["contrast"]} | {c["mean_difference_m"]:.3f} | [{a:.3f}, {b:.3f}] |')
 lines += ['', 'Only five development and three same-object held-out sequences support these descriptive intervals. The iid sequence assumption is weak because sequences share a tank and targets. Do not present these intervals as population-level significance. No parameters were selected using these held-out scores.',
 '', '## Post-hoc bearing-direction diagnostic','',
 '[Bearing diagnostic](bearing_sign_diagnostic.json) prescribes a reversal of estimated x before proper SE(2) scoring. It is an algebraic diagnostic, not a newly run or validated estimator. It improves some recordings and worsens others. No best-sign score replaces the frozen results. This check was added after seeing mirrored-looking paths, so it is explicitly exploratory; these held-out recordings cannot be reused as untouched tests for a future adapter correction.', '', '## Remaining work before a PhD-level validation claim', '', '1. Resolve physical bearing handedness with an off-axis target and resolve the tilted imaging geometry in the state/measurement model, or acquire a truly compatible planar benchmark. Adding reference orientations to estimation must be labeled sensor-aided and must use a legitimate measured input.',
 '2. Develop and calibrate correspondence rejection and loop verification on development data, then freeze a new protocol and use fresh held-out objects/sites. These three test recordings are now consumed for any future tuning.',
 '3. Compare against an independently implemented published sonar SLAM baseline with matching sensor assumptions; evaluate map consistency and long revisits, not just short object scans.',
 '4. Validate simulator acoustics separately using controlled geometry, image statistics, and repeat measurements. This experiment tests SLAM on external imagery; it does not validate sound propagation.',
 '5. Test actual synchronized IMU/sonar data for an inertial-fusion claim. None was synthesized from truth here.',
 '', '## Reproducibility and sources','',
 '[Protocol](PROTOCOL.md), [commands](README.md), [all numerical results](all_metrics.csv), [sequence bootstrap](sequence_bootstrap.json), [timestamp sensitivity](time_shift_sensitivity.json). Per-run folders retain trajectories, loop measurements, input hashes, and source snapshots.',
 '', 'Dataset: Dahn et al., *An Acoustic and Optical Dataset for the Perception of Underwater Unexploded Ordnance (UXO)*, OCEANS 2024, [DOI](https://doi.org/10.1109/OCEANS55160.2024.10754316), [data](https://zenodo.org/records/13778485), [publisher code](https://github.com/dfki-ric/uxo-dataset2024). This is not a verified Negahdaripour/Woods benchmark. No professor email was sent.']
 (HERE/'REPORT.md').write_text('\n'.join(lines)+'\n')
 print(df[df.stride==1][['sequence','split','raw_frames','odometry_ate_m','loop_ate_m','stationary_ate_m']].to_string(index=False))
if __name__=='__main__':main()
