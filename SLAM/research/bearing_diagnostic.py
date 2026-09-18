"""Post-hoc scoring diagnostic, NOT an alternative validated estimator run.
Reflection is prescribed to simulate a bearing-sign convention reversal, never
selected or fitted by the trajectory scorer. Original frozen results stay intact.
"""
from evaluate import *
HERE=Path(__file__).resolve().parent
rows=[]
for split in ['development','heldout']:
 for p in sorted((HERE/'runs'/split).glob('*_stride1/keyframes.csv')):
  d=pd.read_csv(p);truth=d[['gt_x','gt_y']].to_numpy();xy=d[['after_x','after_y']].to_numpy()
  _,e=score(xy,truth);xy[:,0]*=-1;_,re=score(xy,truth)
  rows.append(dict(sequence=p.parent.name,split=split,original_ate_m=rmse(e),prescribed_bearing_reversal_ate_m=rmse(re)))
(HERE/'bearing_sign_diagnostic.json').write_text(json.dumps(rows,indent=2))
print(json.dumps(rows,indent=2))
