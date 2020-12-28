#!/usr/bin/env python
import numpy as np

def distance(p1,p2):
    return np.hypot(p1[0]-p2[0],p1[1]-p2[1])
