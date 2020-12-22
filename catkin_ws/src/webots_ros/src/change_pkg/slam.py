#!/usr/bin/env python
import tf
import rospy
import change_pkg.utils as utils
import math

class Slam:
    def __init__(self, robot):
        self.robot=robot

    def broadcast_transform(self):
        br = tf.TransformBroadcaster(queue_size=100)
        od = self.robot.odometry
        translation=(od.x,od.y,0)
        iu=self.robot.sensors.inertial_unit.value
        rotation=(iu.x,iu.y,iu.z,iu.w)
        br.sendTransform(translation,rotation,rospy.Time().now(),"base_link", "odom")
        br.sendTransform((0,0,0),(0, 0, 0, 1),rospy.Time().now(), "/"+self.robot.name+"/"+self.robot.sensors.lidar.name,"base_link")
