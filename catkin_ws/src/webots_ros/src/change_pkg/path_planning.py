import rospy
import math
from change_pkg.vision import *
import numpy as np
import change_pkg.clustering as clst

class Path_planner:

    """Docstring for Path_planner. """

    def __init__(self, robot, resolution=1, radius=5):
        """Accepts a robot and an.__robot.odometry object

        :robot: The robot this module belongs to
        .__robot.odometry: The odometry object used to estimate the position

        """
        self.__robot = robot
        self.target = [self.__robot.odometry.x,self.__robot.odometry.y]
        self.resolution = resolution
        self.radius=radius
        self.KP = 15
        self.ETA = 25
        self.OSCILLATIONS_DETECTION_LENGTH = 3
        self.SECURITY_DISTANCE = 2.0
        self.MAX_DEVIATION = 45
        self.LIDAR_THRESHOLD = 0.2 # Percentage of lidar sensor values to discard



    def __abs_cartesian_to_polar(self, p):
        """Accepts a point given in cartesian coordinates relative to the world
        frame of reference and converts it to polar coordinates in the robot
        frame of reference based on the current position estimate

        :p:   Cartesian coordinates in the (x, y) format
        :returns:  Cartesian coordinates in the (Rho, Theta) format

        """
        x = p[0] - self.__robot.odometry.x
        y = p[1] - self.__robot.odometry.y
        return (np.hypot(x,y), -180/math.pi*math.atan2(x,y)-self.__robot.odometry.theta)


    def __polar_to_abs_cartesian(self, p):
        # TODO odometry is not used
        """Accepts a point given in polar coordinates relative to the robot
        frame of reference and converts it to cartesian coordinates in a wolrd
        frame of reference based on the current position estimate

        :p:   Polar coordinates in the (Rho, Theta) format
        :returns:       Cartesian coordinates

        """
        theta = self.__robot.odometry.theta
        x = self.__robot.odometry.x
        y = self.__robot.odometry.y
        angle=p[1]+theta
        return(p[0]*math.sin(math.pi*angle/180),p[0]*math.cos(math.pi*angle/180))

    def find_clusters(self):
        # TODO find a more suitable module for this function
        """Based on the self.people_coords finds clusters and returns them.
        This may not be the best place for this function, may move to vision
        module or a specific "reasoning" object.

        :returns: Clusters, either as the points in each cluster as a list of
            lists, or a centroid.

        """
        # TODO find reasonable parameters for the Density Based Scan clustering
        # algorithm
        pass


    def set_target(self, target):
        """Sets the target point for the movement algorithm

        :target: Target coordinate
        :returns: None

        """
        self.target = target

    def movement_distance(self):
        size=len(self.__robot.sensors.lidar.value)
        sup_limit=math.ceil(size/2)
        inf_limit=sup_limit-1
        if self.__robot.sensors.lidar.value[inf_limit][0]>self.__robot.sensors.lidar.value[sup_limit][0]:
            max_distance=self.__robot.sensors.lidar.value[inf_limit][0]
        else:
            max_distance=self.__robot.sensors.lidar.value[sup_limit][0]    
        # y = 0.48 + (MAX - 0.48)/(1 + (x/30)**1.13)
        max_distance_allowed=0.48 + ( max_distance - 0.48)/(1 + (abs(self.target_angle())/30)**1.13)
        distance = min(max_distance_allowed,self.target_distance())
        rospy.logerr("x: {} y:{} max:{}".format(abs(self.target_angle()),distance,max_distance))
        return distance

    def is_obstacle(self, distance):
        return distance < self.SECURITY_DISTANCE   


    def next_step_direction(self):
        """ 
        Using the potential fields method computes the movement direction for
        the next step towards the target 

        :returns: rotation angle
        """
        target_angle = self.__abs_cartesian_to_polar(self.target)[1]       
        direction = target_angle
        size=len(self.__robot.sensors.lidar.value)
        inf_limit=math.ceil(size*self.LIDAR_THRESHOLD)
        sup_limit=size-inf_limit
        obstacles = [ p for p in self.__robot.sensors.lidar.value[inf_limit:sup_limit] if self.is_obstacle(p[0]) ]
        closest_obstacle = (999,None)
        for o in obstacles:
            if o[0] < closest_obstacle[0]:
                closest_obstacle = o
        if closest_obstacle[1] != None:
            attractive_potential = self.KP * np.hypot(self.__robot.odometry.x \
                    - self.target[0], self.__robot.odometry.y - self.target[1])
            repulsive_potential = self.ETA * 1/closest_obstacle[0]**2
            direction = 180/math.pi*math.atan2( 
                        attractive_potential*math.sin(target_angle/180*math.pi) - repulsive_potential*math.sin(closest_obstacle[1]/180*math.pi),
                        attractive_potential*math.cos(target_angle/180*math.pi) - repulsive_potential*math.cos(closest_obstacle[1]/180*math.pi) )
            direction = max(-self.MAX_DEVIATION, min(self.MAX_DEVIATION, direction))
        return direction

    def target_distance(self):
        dist, angle = self.__abs_cartesian_to_polar(self.target)
        return dist

    def target_angle(self):
        dist, angle = self.__abs_cartesian_to_polar(self.target)
        return angle    
