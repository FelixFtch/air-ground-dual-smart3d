"""Geometric lookahead on the actual planner path, independent of CARLA."""
import math
import numpy as np


def lookahead(position, path, distance=.6):
    p=np.asarray(position,dtype=float); points=np.asarray(path,dtype=float)
    if len(points)<2:return points[-1] if len(points) else p
    best=None
    for i,(a,b) in enumerate(zip(points[:-1],points[1:])):
        ab=b-a;t=float(np.clip(np.dot(p-a,ab)/(np.dot(ab,ab)+1e-12),0,1));q=a+t*ab
        value=float(np.linalg.norm(p-q))
        if best is None or value<best[0]:best=(value,i,q)
    _,i,q=best
    for b in points[i+1:]:
        gap=float(np.linalg.norm(b-q))
        if gap>=distance:return q+(b-q)*(distance/max(gap,1e-12))
        distance-=gap;q=b
    return points[-1]


def bounded_step(position, yaw, target, speed, max_speed, acceleration, yaw_rate, dt):
    p=np.array(position,dtype=float);v=np.array(target,dtype=float)-p;d=float(np.linalg.norm(v))
    angle=math.atan2(v[1],v[0]) if np.linalg.norm(v[:2])>.05 else yaw
    err=(angle-yaw+math.pi)%(2*math.pi)-math.pi
    yaw_next=yaw+max(-yaw_rate*dt,min(yaw_rate*dt,err))
    desired=max_speed*max(0.,math.cos(err)) if abs(err)<.65 else 0.
    new_speed=max(0.,min(desired,speed+acceleration*dt))
    step=min(d,new_speed*dt)
    return (p+v*step/max(d,1e-12)).tolist(),yaw_next,new_speed
