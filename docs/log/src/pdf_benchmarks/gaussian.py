#!/usr/bin/env python

import sys
import math
from scipy.stats import multivariate_normal

for _ in range(int(sys.argv[2])):
    if int(sys.argv[1]):
        var = multivariate_normal(mean=[0,0], cov=[[2,0],[0,10]])
        a=(var.pdf([5,(0-3)%360]))
    else:
        a=0.01 + 1/(1+math.hypot((abs(0-5)/2),(3)/3))
    
