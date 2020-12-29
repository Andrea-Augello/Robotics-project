import copy
import math
import rospy

import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import gaussian_filter
from scipy.stats import multivariate_normal


# Parameters
EXTEND_AREA = 100.0  # [m] grid map extended length
SIM_TIME = 50.0  # simulation time [s]
DT = 0.1  # time tick [s]
MAX_RANGE = 5.59  # maximum observation range
RANGE_STD = 1.0  # standard deviation for observation gaussian distribution

# grid map param
XY_RESOLUTION = 0.25  # xy grid resolution
MIN_X = -50.0
MIN_Y = -50.0
MAX_X = 50.0
MAX_Y = 50.0

show_animation = True

class GridMap:

    def __init__(self, robot, xy_resolution = 0.25, min_x =-10, min_y = -10, max_x=10, max_y=10):
        self.__robot = robot
        self.xy_resolution = xy_resolution
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y
        self.x_w = int(round((self.max_x - self.min_x)
                                / self.xy_resolution))
        self.y_w = int(round((self.max_y - self.min_y)
                                / self.xy_resolution))
        self.data =  [[1.0 for _ in range(self.y_w)]
                     for _ in range(self.x_w)]
        self.normalize_probability()

    def normalize_probability(self):
        rospy.logerr("NORMALIZING PROBABILITIES")
        sump = sum([sum(i_data) for i_data in self.data])

        for ix in range(self.x_w):
            for iy in range(self.y_w):
                self.data[ix][iy] /= sump

    def draw_heat_map(self):
        rospy.logerr("DRAWING HEAT MAP")
        mx, my = self.calc_grid_index()
        max_value = max([max(i_data) for i_data in self.data])
        plt.pcolor(mx, my, self.data,vmax=max_value,cmap=plt.cm.get_cmap("Blues"))
        plt.axis("equal")
        plt.show()

    def calc_grid_index(self):
        mx, my = np.mgrid[slice(self.min_x - self.xy_resolution / 2.0,
                                self.max_x + self.xy_resolution / 2.0,
                                self.xy_resolution),
                        slice(self.min_y - self.xy_resolution / 2.0,
                                self.max_y + self.xy_resolution / 2.0,
                                self.xy_resolution)]

        return mx, my

    def observation_update(self, z, std=1):
        rospy.logerr("UPDATING MAP")
        for ix in range(self.x_w):
            for iy in range(self.y_w):
                prob = 0
                for iz in range(len(z)):
                    prob += self.calc_gaussian_observation_pdf(
                        z, iz, ix, iy, std)
                self.data[ix][iy] *= prob
        self.normalize_probability()
        self.draw_heat_map()

    def calc_gaussian_observation_pdf(self, z, iz, ix, iy, std=1):
        # predicted range
        x = ix * self.xy_resolution + self.min_x
        y = iy * self.xy_resolution + self.min_y

        o_distance, o_angle = z[iz]
        p_distance, p_angle = self.__robot.odometry.abs_cartesian_to_polar((x,y))
        angle_diff = (o_angle-p_angle)%360
        angle_diff = angle_diff if angle_diff < 180 else 360-angle_diff
        # likelihood
        # var = multivariate_normal(mean=[o_distance,0], cov=[[2,0],[0,10]])
        #return (var.pdf([p_distance,(o_angle-p_angle)%360]))
        return 0.01 + 1/(1+math.hypot((abs(o_distance-p_distance)/2),(angle_diff)/3))
