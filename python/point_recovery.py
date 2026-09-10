"""Known-correspondence range/bearing recovery with explicit finite-difference sensitivity."""
import numpy as np


def observe(point, poses):
    values=[]
    for position,axes in poses:
        local=(np.asarray(point)-position) @ axes
        values.extend([np.linalg.norm(local),np.arctan2(local[0],local[1])])
    return np.array(values)


def jacobian(point,poses):
    h=1e-5;directions=np.eye(3)*h
    return np.column_stack([(observe(point+d,poses)-observe(point-d,poses))/(2*h) for d in directions])


def solve(measured,poses,initial,range_sigma=.003,bearing_sigma=np.radians(.05)):
    p=np.array(initial,dtype=float);scale=np.tile([range_sigma,bearing_sigma],len(poses))
    for _ in range(40):
        residual=(observe(p,poses)-measured)/scale
        jac=jacobian(p,poses)/scale[:,None]
        delta=np.linalg.lstsq(jac,-residual,rcond=1e-10)[0]
        if np.linalg.norm(delta)>1:delta/=np.linalg.norm(delta)
        p+=delta
        if np.linalg.norm(delta)<1e-8:break
    return p
