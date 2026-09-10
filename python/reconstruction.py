"""Segmentation-dependent sonar feasible volume; no access to truth during carving."""
import numpy as np
import sonar


def segment(image, fraction=0.03):
    highlight=(image>fraction*image.max()) & (image>0)
    # Conservative ambiguity: any farther cell on a bearing with a highlight may
    # be hidden. This intentionally does NOT claim dark water is a measured shadow.
    shadow=np.maximum.accumulate(highlight,axis=1) & ~highlight
    return highlight,shadow


def grid(centre=(0,4,0), half=0.7, step=0.08):
    axis=np.arange(-half+step/2,half,step)
    points=np.stack(np.meshgrid(axis,axis,axis,indexing='ij'),axis=-1).reshape(-1,3)+centre
    return points,(len(axis),)*3


def metrics(kept, truth, voxel_size):
    tp=np.count_nonzero(kept & truth);fp=np.count_nonzero(kept & ~truth);fn=np.count_nonzero(~kept & truth)
    return dict(iou=tp/max(tp+fp+fn,1),normalized_volume_error=(fp-fn)/max(truth.sum(),1),
                false_positive_m3=fp*voxel_size**3,false_negative_m3=fn*voxel_size**3)


def orbit_poses(count, centre=np.array([0.,4.,0.]), distance=4., roll=True):
    for angle in np.linspace(0,2*np.pi,count,endpoint=False):
        position=centre+distance*np.array([np.sin(angle),-np.cos(angle),0.15*np.sin(2*angle)])
        forward=centre-position;forward/=np.linalg.norm(forward)
        right=np.cross(forward,[0,0,1]);right/=np.linalg.norm(right)
        up=np.cross(right,forward);r=0.7*np.sin(3*angle) if roll else 0
        yield position,np.column_stack((np.cos(r)*right+np.sin(r)*up,forward,-np.sin(r)*right+np.cos(r)*up))


def reconstruct(images,poses,sim,points,threshold=0.03,tolerance=1):
    kept=np.ones(len(points),bool)
    for image,(position,axes) in zip(images,poses):
        highlight,shadow=segment(image,threshold)
        kept=sonar.carve(points,sim,highlight,shadow,position,axes,kept,tolerance,tolerance)
    return kept


def surface_mesh(kept,points,shape,step):
    """Expose only voxel faces bordering empty space, sharing corner vertices."""
    occupied=kept.reshape(shape);vertices=[];faces=[];indices={}
    corners=np.array([[x,y,z] for z in (-1,1) for y in (-1,1) for x in (-1,1)])*step/2
    quads=[(0,2,3,1),(4,5,7,6),(0,1,5,4),(2,6,7,3),(0,4,6,2),(1,3,7,5)]
    offsets=[(0,0,-1),(0,0,1),(0,-1,0),(0,1,0),(-1,0,0),(1,0,0)]
    for index in np.argwhere(occupied):
        centre=points[np.ravel_multi_index(index,shape)]
        for q,offset in zip(quads,offsets):
            neighbor=index+offset
            if np.all(neighbor>=0) and np.all(neighbor<shape) and occupied[tuple(neighbor)]:continue
            ids=[]
            for i in q:
                p=tuple(np.round(centre+corners[i],10))
                if p not in indices:indices[p]=len(vertices);vertices.append(p)
                ids.append(indices[p])
            faces.extend([(ids[0],ids[1],ids[2]),(ids[0],ids[2],ids[3])])
    return np.array(vertices),np.array(faces,dtype=int)
