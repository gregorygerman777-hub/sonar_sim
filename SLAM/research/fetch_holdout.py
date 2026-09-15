"""Fetch complete first solid block and selected files; verify each member CRC.
The sparse archive is partial, NOT a verified copy of the full release archive.
"""
import concurrent.futures, pathlib, urllib.request, time, struct, zlib, json
import py7zr
ROOT=pathlib.Path(__file__).resolve().parents[2]
out=ROOT/'data_external/dfki_uxo_2024/holdout';out.mkdir(exist_ok=True)
p=out/'partial_recordings.7z'
url='https://zenodo.org/records/13778485/files/data_export_recordings.7z'
if not p.exists():
 def get_range(a,b):
  req=urllib.request.Request(url,headers={'Range':f'bytes={a}-{b}'})
  with urllib.request.urlopen(req,timeout=60) as r:
   assert r.status==206
   data=r.read();assert len(data)==b-a+1
   return data
 head=get_range(0,31)
 offset,length,_=struct.unpack('<QQI',head[12:32])
 end=32+offset+length
 # Encoded header data immediately precedes the tiny next-header record.
 tail_start=max(32,32+offset-2**20)
 tail=get_range(tail_start,end-1)
 with p.open('wb') as f:
  f.write(head);f.seek(tail_start);f.write(tail)
size=1883555022+32;block=32*1024*1024
jobs=[(a,min(size-1,a+block-1)) for a in range(32,size,block)]
def fetch(ab):
 a,b=ab
 for attempt in range(4):
  try:
   req=urllib.request.Request('https://zenodo.org/records/13778485/files/data_export_recordings.7z',headers={'Range':f'bytes={a}-{b}'})
   with urllib.request.urlopen(req,timeout=60) as r:
    assert r.status==206
    data=r.read();assert len(data)==b-a+1
   with p.open('r+b') as f:f.seek(a);f.write(data)
   return
  except Exception:
   if attempt==3:raise
   time.sleep(2)
with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
 for i,_ in enumerate(pool.map(fetch,jobs)):print('block chunk',i+1,'/',len(jobs),flush=True)
seqs=['2023-09-20_171105','2023-09-20_172513','2023-09-20_172851']
with py7zr.SevenZipFile(p) as a:
 members=[f for f in a.files if not f.is_directory and any('/'+s+'/' in f.filename for s in seqs) and ('/aris_raw/' in f.filename or f.filename.endswith(('.csv','.yaml','.txt')))]
 names=[f.filename for f in members];crc={f.filename:f.crc32 for f in members}
 a.extract(path=out,targets=names)
for name in names:
 assert zlib.crc32((out/name).read_bytes())==crc[name],name
(out/'extraction_receipt.json').write_text(json.dumps(dict(source='https://zenodo.org/records/13778485/files/data_export_recordings.7z',scope='First compressed solid block only; full archive checksum NOT verified',sequences=seqs,members_crc32=crc,all_member_crcs_verified=True),indent=2))
print('VERIFIED',len(names),'files',flush=True)
