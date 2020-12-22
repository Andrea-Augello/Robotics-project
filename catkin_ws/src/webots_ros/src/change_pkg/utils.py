#!/usr/bin/env python
import math
from scipy.spatial.transform import Rotation

def quaternion_to_degree(q):
    rot = Rotation.from_quat([q.x,q.y,q.z,q.w])
    rot_euler = rot.as_euler('xyz', degrees=True) 
    return 180*rot_euler[2]/math.pi

def euler_to_quaternion(e):
    rot = Rotation.from_euler('xyz', e, degrees=True)
    return rot.as_quat()

