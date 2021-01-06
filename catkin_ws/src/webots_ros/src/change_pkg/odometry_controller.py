#!/usr/bin/env python
import change_pkg.utils as utils
import rospy
import os



class Odometry:
    def __init__(self):
        self.node_name = 'odometry_node'
        self.node_name = 'odom'
        self.time_step = 32
        self.x=2 
        self.y=-2.55
        self.history=[((self.x,self.y),0)]
        self.theta=0

    def init(self):
        rospy.init_node(self.node_name, anonymous=True)
        rospy.loginfo('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
        rospy.loginfo('Time step: ' + str(self.time_step))
        self.start()
    

    def start(self):
        while True:
            x=0
            y=0
            theta=0
            utils.publish_odometry("/"+self.node_name+"/"+self.topic_name, x,y,theta)  
