import copy
import math
import cv2
import change_pkg.utils as utils
import change_pkg.clustering as clst
import matplotlib.pyplot as plt
import numpy as np
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

    def __init__(self, robot, xy_resolution = 0.25, min_x =-10, min_y = -10, max_x=10, max_y=10, show_map=True):
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
        self.show_map=show_map

    def reset(self):
        self.data =  [[1.0 for _ in range(self.y_w)]
                     for _ in range(self.x_w)]

    def normalize_probability(self):
        sump = sum([sum(i_data) for i_data in self.data])

        for ix in range(self.x_w):
            for iy in range(self.y_w):
                self.data[ix][iy] /= sump

    def draw_heat_map(self):
        mx, my = self.calc_grid_index()
        max_value = max([max(i_data) for i_data in self.data])
        plt.pcolor(mx, my, self.data,vmax=max_value,cmap=plt.cm.get_cmap("Blues"))
        plt.axis("equal")
        plt.show()

    def draw_clusters(self,clusters):
        x=[]
        y=[]
        x_contour=[]
        y_contour=[]

        for cluster in clusters:
            x.append(cluster['center'][0])
            y.append(cluster['center'][1])
            for contour in cluster['contour']:
                x_contour.append(contour[0])
                y_contour.append(contour[1])

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))
        fig.suptitle('People recognition')
        mx, my = self.calc_grid_index()
        max_value = max([max(i_data) for i_data in self.data])
        ax1.pcolor(mx, my, self.data,vmax=max_value,cmap=plt.cm.get_cmap("Blues"))
        ax1.axis("equal")
        ax1.set_xlabel('Position (m)')
        ax1.set_ylabel('Position (m)')
        ax2.scatter(x, y, s=20)
        ax2.scatter(x_contour, y_contour, s=10)
        ax2.set_xlabel('Position (m)')
        plt.xlim([ self.max_x,self.min_x])
        plt.ylim([self.min_y, self.max_y])
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
        utils.loginfo("UPDATING MAP")
        for ix in range(self.x_w):
            for iy in range(self.y_w):
                prob = 0
                for iz in range(len(z)):
                    prob += self.calc_gaussian_observation_pdf(
                        z, iz, ix, iy, std)
                self.data[ix][iy] *= prob
        self.normalize_probability()
        #return self.find_clusters_2()
        return self.find_centroid()

    def calc_gaussian_observation_pdf(self, z, iz, ix, iy, std=1):
        # predicted range
        x = ix * self.xy_resolution + self.min_x
        y = iy * self.xy_resolution + self.min_y

        o_distance, o_angle = z[iz]
        p_distance, p_angle = self.__robot.odometry.abs_cartesian_to_polar((x,y))
        angle_diff = (-o_angle-p_angle)%360
        angle_diff = angle_diff if angle_diff < 180 else 360-angle_diff
        # likelihood
        #var = multivariate_normal(mean=[o_distance,0], cov=[[2,0],[0,10]])
        #return (var.pdf([p_distance,(o_angle-p_angle)%360]))
        return 0.01 + 1/(1+math.hypot((abs(o_distance-p_distance)/2),(angle_diff)/3))



    def find_centroid(self):
        im = np.array(self.data)
        max_value = max([max(i_data) for i_data in self.data])
        min_value = min([min(i_data) for i_data in self.data])
        im = im-min_value
        im = im /(max_value-min_value)
        im = 255 * im
        im = im.astype(np.uint8)
        im = im.T
        #im = np.flip(im,0)
        im=im[:,:,None]
        

        # Calculate centroids

        otsu_threshold, thresh = cv2.threshold( im, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_L1)
        #cv2.drawContours(im, contours, -1, (0,255,0), 3)
        clusters=[]
        for c in contours:
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            cx = self.resize_x(int(M['m10']/M['m00']))
            cy = self.resize_y(int(M['m01']/M['m00']))
            contour_point=[(self.resize_x(x),self.resize_y(y)) for [[x,y]] in c]
            clusters.append(
                    {'center':(cx,cy),
                        'area':M['m00'],
                        'contour':contour_point})
            #cv2.circle(im, (cx,cy), 2, (0,255,0), 1)
        if self.show_map:
            self.draw_clusters(clusters)
        return clusters    


    def find_clusters_2(self):
        # TODO find a more suitable module for this function
        """Based on the self.people_coords finds clusters and returns them.

        :returns: Clusters, either as the points in each cluster as a list of
            lists, or a centroid.

        """
        threshold = max([max(i_data) for i_data in self.data])
        threshold *= 0.9
        point_list=[]
        for i,row in enumerate(self.data):
            for j,col in enumerate(row):
                if self.data[i][j]>threshold:
                    point_list.append((self.resize_x(i),self.resize_y(j)))
        # TODO find reasonable parameters for the Density Based Scan clustering
        # algorithm
        clusters = clst.clustering(point_list, 
                distance_measure=utils.math_distance,
                min_samples=1,
                eps=0.5)
        self.draw_clusters(clusters)
        return None if len(clusters) == 0 else clusters

    def resize_x(self,i):
        return (i*self.xy_resolution)+self.min_x

    def resize_y(self,i):
        return (i*self.xy_resolution)+self.min_y

    def find_clusters(self, polar_coords):
        """Based on the self.people_coords finds clusters and returns them.

        :returns: Clusters, either as the points in each cluster as a list of
            lists, or a centroid.

        """
        cartesian_coords = [ self.__robot.odometry.polar_to_abs_cartesian(p) for p in polar_coords ]
        clusters = clst.clustering(cartesian_coords, 
                distance_measure=utils.math_distance,
                min_samples=2,
                eps=2.5)
        return None if len(clusters) == 0 else clusters[0]
