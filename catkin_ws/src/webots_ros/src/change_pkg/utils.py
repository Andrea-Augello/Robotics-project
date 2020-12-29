#!/usr/bin/env python
import numpy as np
import rospy
import math

def distance(p1,p2):
    return np.hypot(p1[0]-p2[0],p1[1]-p2[1])

def math_distance(p1,p2):
    return math.hypot(p1[0]-p2[0],p1[1]-p2[1])

def loginfo(text):
    rospy.logwarn(text)

def logerr(text):
    rospy.logerr(text)

def debug(text):
    rospy.logwarn(text)        
