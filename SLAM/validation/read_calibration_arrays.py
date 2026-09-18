"""Restricted conversion of the publisher's small numeric-array pickle files."""
import pickle,json
from pathlib import Path
import numpy as np
class NumericArraysOnly(pickle.Unpickler):
 def find_class(self,module,name):
  allowed={('numpy.core.multiarray','_reconstruct'):np._core.multiarray._reconstruct,('numpy','ndarray'):np.ndarray,('numpy','dtype'):np.dtype}
  if (module,name) not in allowed:raise pickle.UnpicklingError(f'Forbidden global {module}.{name}')
  return allowed[module,name]
root=Path(__file__).resolve().parents[2]/'data_external/sonar_extrinsics'
for path in root.glob('*.pkl'):
 if path.stat().st_size>100000:raise ValueError('Unexpected pickle size')
 with path.open('rb') as f:data=NumericArraysOnly(f).load()
 result={}
 for k,v in data.items():
  if not isinstance(k,str) or not isinstance(v,np.ndarray) or v.dtype.kind not in 'fi':raise ValueError('Expected numeric arrays')
  result[k]=v.tolist();print(path.name,k,v.shape)
 path.with_suffix('.json').write_text(json.dumps(result))
