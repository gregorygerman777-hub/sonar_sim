"""Procedural, metre-scale teaching targets. No external or generated imagery."""
from pathlib import Path
import json
import numpy as np


def box(lo, hi):
    vertices=np.array([[x,y,z] for z in (lo[2],hi[2]) for y in (lo[1],hi[1]) for x in (lo[0],hi[0])])
    quads=[(0,2,3,1),(4,5,7,6),(0,1,5,4),(2,6,7,3),(0,4,6,2),(1,3,7,5)]
    faces=np.array([(q[0],q[i],q[i+1]) for q in quads for i in (1,2)])
    return vertices,faces


def radial_target(coral=False, rings=10, segments=20):
    vertices=[[0,0,-0.5]]
    for j in range(1,rings):
        phi=-np.pi/2+np.pi*j/rings
        for i in range(segments):
            theta=2*np.pi*i/segments
            r=0.5*(1+0.18*np.sin(5*theta)*np.cos(3*phi)**2) if coral else 0.5
            vertices.append([r*np.cos(phi)*np.cos(theta),r*np.cos(phi)*np.sin(theta),r*np.sin(phi)])
    top=len(vertices);vertices.append([0,0,0.5]);faces=[]
    for i in range(segments):
        n=(i+1)%segments
        faces.append((0,1+n,1+i))
        for j in range(rings-2):
            a=1+j*segments+i;b=1+j*segments+n;c=b+segments;d=a+segments
            faces.extend([(a,b,c),(a,c,d)])
        faces.append((top,1+(rings-2)*segments+i,1+(rings-2)*segments+n))
    return np.array(vertices),np.array(faces)


def pipe(segments=24):
    # Closed annular pipe, axis X. Both inner and outer walls participate in ray tests.
    vertices=np.array([[x,r*np.cos(t),r*np.sin(t)] for x in (-0.6,0.6)
                      for r in (0.16,0.24) for t in np.arange(segments)*2*np.pi/segments])
    faces=[]
    for i in range(segments):
        n=(i+1)%segments
        for q in ((segments+i,segments+n,3*segments+n,3*segments+i),
                  (i,2*segments+i,2*segments+n,n),
                  (i,n,segments+n,segments+i),
                  (2*segments+i,3*segments+i,3*segments+n,2*segments+n)):
            faces.extend([(q[0],q[1],q[2]),(q[0],q[2],q[3])])
    return vertices,np.array(faces)


def table():
    parts=[box((-0.6,-0.4,0.32),(0.6,0.4,0.45))]
    for x in (-0.48,0.48):
        for y in (-0.28,0.28):
            parts.append(box((x-0.06,y-0.06,-0.45),(x+0.06,y+0.06,0.32)))
    vertices=[];faces=[]
    for v,f in parts: faces.extend(f+len(vertices));vertices.extend(v)
    return np.array(vertices),np.array(faces)


def audit(vertices, faces):
    edges={};signed={}
    for face in faces:
        for a,b in zip(face,np.roll(face,-1)):
            edge=tuple(sorted((int(a),int(b))))
            edges[edge]=edges.get(edge,0)+1;signed[edge]=signed.get(edge,0)+(1 if a<b else -1)
    volume=np.sum(np.sum(vertices[faces[:,0]]*np.cross(vertices[faces[:,1]],vertices[faces[:,2]]),axis=1))/6
    return dict(triangle_count=len(faces),dimensions_m=np.ptp(vertices,axis=0).tolist(),
                boundary_edges=sum(n==1 for n in edges.values()),nonmanifold_edges=sum(n>2 for n in edges.values()),
                inconsistent_winding_edges=sum(s!=0 for s in signed.values()),signed_volume_m3=float(volume),
                self_intersections='not exhaustively checked; table consists of closed boxes meeting at legs')


def generate(directory='assets'):
    path=Path(directory);path.mkdir(parents=True,exist_ok=True);records={}
    targets={'box':box((-0.5,-0.5,-0.5),(0.5,0.5,0.5)), 'sphere':radial_target(),
             'coral_rock':radial_target(True),'industrial_pipe':pipe(),'concave_table':table()}
    for name,(v,f) in targets.items():
        with (path/(name+'.obj')).open('w') as out:
            out.write('# Procedural educational target; metres, X right Y forward Z up\n')
            for p in v: out.write('v '+' '.join(map(str,p))+'\n')
            for p in f+1: out.write('f '+' '.join(map(str,p))+'\n')
        records[name]=dict(source='python/mesh_assets.py, deterministic procedural geometry',license='MIT',units='metres',
             coordinate_convention='right-handed X starboard Y forward Z up',reflectivity='default 0.8 diffuse coefficient; uncalibrated',**audit(v,f))
    (path/'manifest.json').write_text(json.dumps(records,indent=2))
    return targets

if __name__=='__main__':
    generate()
    print(Path('assets/manifest.json').read_text())
