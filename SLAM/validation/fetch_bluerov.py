"""Public dataset acquisition; no trajectory scores or tuning."""
from pathlib import Path
import gdown,hashlib,json
root=Path(__file__).resolve().parents[2]/'data_external/bluerov_uuv';root.mkdir(exist_ok=True)
files={'TransMatrix.yaml':'1lSdWnZAvlCta3drdO3EzYzelRchuycDI','line.bag':'1zXpRlh0Py1Kpo5hxmEw1c-tEuJFNNgMf','circle.bag':'1zaARZei0Ia0-Ub7ZfkIIesQiDM4kUjie','L-shape.bag':'1-9RKfiI95o-LDyXq6VYM7Hbu-J6295Ek'}
for name,id in files.items():
 p=root/name
 if not p.exists():gdown.download(id=id,output=str(p),quiet=False)
 if not p.exists():raise RuntimeError(f'Failed to download {name}')
 print(name,p.stat().st_size,flush=True)
(root/'source.json').write_text(json.dumps(dict(source='https://github.com/hwgao1101/Sonar_Based_UUV_DataSet',files=files,split='line: admission/calibration development; circle and L-shape: reserved, no trajectory scoring until protocol freeze'),indent=2))
