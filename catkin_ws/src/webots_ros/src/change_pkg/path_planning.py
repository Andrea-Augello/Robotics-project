import rospy
import math
from change_pkg.vision import *
import numpy as np
import change_pkg.clustering as clst

class Path_planner:

    """Docstring for Path_planner. """

    def __init__(self, robot, resolution=1, radius=5):
        """Accepts a robot and an odometry object

        :robot: The robot this module belongs to
        :odometry: The odometry object used to estimate the position

        """
        self.__robot = robot
        self.people_coords=[]
        self.target = [self.__robot.odometry.x,self.__robot.odometry.y]
        self.resolution = resolution
        self.radius=radius
        self.KP = 5
        self.ETA = 100
        self.OSCILLATIONS_DETECTION_LENGTH = 3



    def __abs_cartesian_to_polar(self, p):
        """Accepts a point given in cartesian coordinates relative to the world
        frame of reference and converts it to polar coordinates in the robot
        frame of reference based on the current position estimate

        :p:   Cartesian coordinates in the (x, y) format
        :returns:  Cartesian coordinates in the (Rho, Theta) format

        """
        x = p[0] - self.odometry.x
        y = p[1] - self.odometry.y
        return (np.hypot(x,y), math.atan2(y,x)-self.odometry.theta)


    def __polar_to_abs_cartesian(self, p):
        """Accepts a point given in polar coordinates relative to the robot
        frame of reference and converts it to cartesian coordinates in a wolrd
        frame of reference based on the current position estimate

        :p:   Polar coordinates in the (Rho, Theta) format
        :returns:       Cartesian coordinates

        """
        theta = self._odometry.theta
        x = self._odometry.x
        y = self._odometry.y
        angle=p[1]+theta
        return(p[0]*math.sin(math.pi*angle/180),p[0]*math.cos(math.pi*angle/180))

    def update_target_coords(self, new_cords):
        """
        :new_cords: The updated coordinates
        :returns: TODO

        """
        cartesian_coords = [ __polar_to_abs_cartesian(p) for p in new_cords ]
        # TODO some more sophisticated analysis here, e.g. iterative closest
        # point estimate for data points that can be linked with past data,
        # keeping the old position as best guess for points that have been
        # lost. Possibly update position estimate with triangulation
        self.people_coords = cartesain

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


    def next_step_direction(self):
        """ 
        Using the potential fields method  computes the movement direction for
        the next step towards the target 

        :returns: rotation angle
        """
        repulsive_potential = 0
        attractive_potential = self.KP * np.hypot(self.__robot.odometry.x \
                - self.target[0], self.__robot.odometry.y - self.target[1])
        target_angle = self.__abs_cartesian_to_polar(self.target)[1]       
        direction = target_angle
        obstacles = [ p in self.__robot.sensors.lidar.value if p[0] < 5.59]
        closest_obstacle = (999,None)
        for o in obstacles:
            if o[0] < closest_obstacle[0]:
                closest_obstacle = o
        if closest_obstacle[1] != None:
            repulsive_potential = ETA * 1/closest_obstacle[0]
            target_angle = math.atan2( 
                        attractive_potential*math.cos(target_angle) - repulsive_potential*math.cos(closest_obstacle[1]),
                        attractive_potential*math.sin(target_angle) - repulsive_potential*math.sin(closest_obstacle[1]) )
        return target_angle
