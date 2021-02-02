import copy
import math
import cv2
import utils as utils
import clustering as clst
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage.filters import gaussian_filter
from scipy.stats import multivariate_normal
from scipy.spatial import distance

class GridMap:

    def __init__(self, robot, xy_resolution = 0.20, min_x =-10, min_y = -10, max_x=10, max_y=10):
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
        self.show_map=robot.debug_mode
        self.next_label=1
        self.seed_dict={}
        self.alias_dict={}

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


    def draw_heat_map_inverted(self):
        mx, my = self.calc_grid_index()
        max_value = max([max(i_data) for i_data in self.data])
        fig,ax1 = plt.subplots(1,1)
        ax1.pcolor(mx, my, self.data,vmax=max_value,cmap=plt.cm.get_cmap("Blues"))
        ax1.invert_xaxis()
        plt.show()

    def draw_heat_map_inverted_centroids(self, centroids):
        x=[i[0] for i in centroids]
        y=[i[1] for i in centroids]
        mx, my = self.calc_grid_index()
        max_value = max([max(i_data) for i_data in self.data])
        fig,ax1 = plt.subplots(1,1)
        ax1.pcolor(mx, my, self.data,vmax=max_value,cmap=plt.cm.get_cmap("Blues"))
        ax1.scatter(x,y,color='y')
        ax1.invert_xaxis()
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
        plt.xlim([self.min_x, self.max_x])
        plt.ylim([self.min_y, self.max_y])
        ax1.invert_xaxis()
        ax2.invert_xaxis()
        plt.show()

    def calc_grid_index(self):
        mx, my = np.mgrid[slice(self.min_x - self.xy_resolution / 2.0,
                                self.max_x + self.xy_resolution / 2.0,
                                self.xy_resolution),
                        slice(self.min_y - self.xy_resolution / 2.0,
                                self.max_y + self.xy_resolution / 2.0,
                                self.xy_resolution)]

        return mx, my

    def observation_update(self, z):
        utils.loginfo("UPDATING MAP")
        noise = 0.2/((self.max_x - self.min_x)*(self.max_y - self.min_y)/ self.xy_resolution**2)
        self.data = gaussian_filter(self.data, sigma=3)
        if len(z):
            for ix in range(self.x_w):
                for iy in range(self.y_w):
                    prob = 0
                    for iz in range(len(z)):
                        prob += self.calc_gaussian_observation_pdf(
                            z, iz, ix, iy)
                    self.data[ix][iy] *= prob
        # adds noise
        for ix in range(self.x_w):
            for iy in range(self.y_w):
                self.data[ix][iy] += np.random.rand()*noise
        self.normalize_probability()
        '''
        error_old=[min([utils.math_distance(p['center'],self.__robot.odometry.polar_to_abs_cartesian(i)) for i in z]) for p in self.find_centroid()]
        error_new=[min([utils.math_distance(p,self.__robot.odometry.polar_to_abs_cartesian(i)) for i in z]) for p in self.find_centroid_region_growing(z)]
        print("Real: {}\nOld: {}\nError Old: {}\nSum Error Old: {:.2f}\nNew: {}\nError New: {}\nSum Error New: {:.2f}\n{}\n".format(
            [(round(self.__robot.odometry.polar_to_abs_cartesian(i)[0],2),round(self.__robot.odometry.polar_to_abs_cartesian(i)[1],2)) for i in z],
            [(round(i['center'][0],2),round(i['center'][1],2)) for i in self.find_centroid()],
            [round(i,2) for i in error_old],
            sum(error_old),
            [(round(i[0],2),round(i[1],2)) for i in self.find_centroid_region_growing(z)],
            [round(i,2) for i in error_new],
            sum(error_new),
            100*"-"))
        '''
        return self.find_centroid_region_growing(z)

    def calc_gaussian_observation_pdf(self, z, iz, ix, iy):
        # predicted range
        # V=[ 0.1627,   -0.0397, -0.0397,    0.0752]
        # IV = [[7.0551039267470060371, 3.7245694932427678148], [3.7245694932427678148, 15.264167671299705881]]
        x = ix * self.xy_resolution + self.min_x
        y = iy * self.xy_resolution + self.min_y

        o_distance, o_angle = z[iz]
        p_distance, p_angle = self.__robot.odometry.abs_cartesian_to_polar((x,y))
        angle_diff = (-o_angle-p_angle)%360
        angle_diff = angle_diff if angle_diff < 180 else 360-angle_diff
        # likelihood
        #var = multivariate_normal(mean=[o_distance,0], cov=[[2,0],[0,10]])
        #return (var.pdf([p_distance,(o_angle-p_angle)%360]))
        return ( 0 if p_distance < 0.5 or abs(angle_diff-180)<45 \
                # else 1/(1+distance.mahalanobis( 
                    # [(o_distance-p_distance),(angle_diff)],
                    # [0,0], IV)**4)) +0.05
                else 1/(1+math.hypot( (o_distance-p_distance)/1.0, (angle_diff*o_distance)/16)**4)) \
                + 0.05 

    def matrix_to_img(self,matrix):
        im = np.array(matrix)
        max_value = max([max(i_data) for i_data in matrix])
        min_value = min([min(i_data) for i_data in matrix])
        im = im-min_value
        r=max_value-min_value
        if r==0:
            r=1
        im = im /r
        im = 255 * im
        im = im.astype(np.uint8)
        im = im.T
        im=im[:,:,None]
        return im


    def find_centroid(self):
        im = self.matrix_to_img(self.data)
        # Calculate centroids

        otsu_threshold, thresh = cv2.threshold( im, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_L1)
        #cv2.drawContours(im, contours, -1, (0,255,0), 3)
        clusters=[]
        for c in contours:
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            mask = np.zeros(im.shape, np.uint8)
            cv2.drawContours(mask, [c],0,255,-1)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(im,mask=mask)
            cx = self.resize_x(max_loc[0])#int(M['m10']/M['m00']))
            cy = self.resize_y(max_loc[1])#int(M['m01']/M['m00']))
            contour_point=[(self.resize_x(x),self.resize_y(y)) for [[x,y]] in c]
            clusters.append(
                    {'center':(cx,cy),
                        'area':M['m00']*self.xy_resolution**2,
                        'contour':contour_point})
            #cv2.circle(im, (cx,cy), 2, (0,255,0), 1)
        if self.show_map:
            self.draw_clusters(clusters)
        return clusters 


    def point_to_coord(self,point):
        x=int((point[0]-self.min_x)/self.xy_resolution)
        y=int((point[1]-self.min_y)/self.xy_resolution)
        return (x,y)

    def coord_to_point(self,coord):
        x=coord[0]*self.xy_resolution+self.min_x
        y=coord[1]*self.xy_resolution+self.min_y
        return (x,y)    

    def find_centroid_region_growing(self,seeds):
        map_cluster,alias=self.region_growing(seeds)

        temp={a: None for a in alias}
        temp.update(self.alias_dict)
        self.alias_dict=temp

        true_alias,false_alias=self.check_alias(alias,map_cluster)
        self.update_seed_dict(true_alias,false_alias)
        map_cluster=self.knn(map_cluster,false_alias)
        centroids=self.get_centroids(map_cluster,true_alias,self.next_label)
        for c in centroids:
            self.seed_dict[c.label]=c.point
        '''
        for c in centroids:
            map_cluster[c.point[0]][c.point[1]]=max([max(i_data) for i_data in map_cluster])+3
        im=self.matrix_to_img(map_cluster)
        cv2.imshow('',im)    
        cv2.waitKey(0)
        '''
        return [self.coord_to_point(i.point) for i in centroids]

    def update_seed_dict(self,true_alias,false_alias):
        delete_list=set()
        for a in false_alias:
            if self.alias_dict[a]<0.1:
                self.alias_dict.pop(a)
        for a in true_alias:
            self.alias_dict.pop(a)
            delete_list.add(a[0])         
        self.alias_dict={(a,b): self.alias_dict[(a,b)] for (a,b) in self.alias_dict if a not in delete_list and b not in delete_list}        
        for s in delete_list:
            self.seed_dict.pop(s)


    def knn(self,map_cluster,false_alias):
        connected_lists = []
        # While some aliases haven't been associated to a cluster
        while len(false_alias) > 0:
            # Take the first unassociated tuple and set it as the starting
            # point for the connected subgraph
            a,b = false_alias.pop()
            connected = set(a,b)
            while(True):
                prev_len = len(false_alias)
                i = 0
                # Loop through the list searching for nodes connected to the
                # subgraph
                while(i < len(false_alias)):
                    a,b = false_alias[i]
                    # If one element of the tuple belongs to the connected
                    # graph, we have an arc to the other one and we can add it
                    # to the subgraph, else we move to the next element in the
                    # list
                    if (a in connected or b in connected):
                        connected.add(a)
                        connected.add(b)
                        false_alias.pop(i)
                    else
                        i+=1
                # If an iteration though the list did not add any new node,
                # then there are no more connected nodes and we can move to the
                # next subgraph
                if prev_len == len(false_alias):
                    break
            connected_lists.append(connected)

        return map_cluster

    def remove_seed(self,seed):
        self.alias_dict={(a,b): self.alias_dict[(a,b)] for (a,b) in self.alias_dict if a!=seed and b!=seed}        
        self.seed_dict.pop(seed)

    def check_alias(self,alias,map_cluster):
        true_alias=[]
        false_alias=[]
        for a in alias:
            old_prob=self.alias_dict[a]
            if old_prob==None:
                prob=math.exp(-utils.math_distance(self.seed_dict[a[0]],self.seed_dict[a[1]]))
            else:
                prob=(old_prob*0.8)/(old_prob*0.8+(1-old_prob)*0.1)
            self.alias_dict[a]=prob
        for a in self.alias_dict.keys():
            if a not in alias:
                old_prob=self.alias_dict[a]
                self.alias_dict[a]=(old_prob*0.2)/(old_prob*0.2+(1-old_prob)*0.9)
            if self.alias_dict[a]>0.9:
                true_alias.append(a)
            else:
                false_alias.append(a)    
           
        return true_alias,false_alias    


    def get_centroids(self,map_cluster,alias,seed_id):
        dictionary={n: [self.x_w,0,self.y_w,0] for n in self.seed_dict.keys()}
        alias_confirmed=set()
        for x,row in enumerate(map_cluster):
            for y,element in enumerate(row):
                if element!=0:
                    seed=element
                    for a in alias:
                        if a[0]==seed:
                            seed=a[1]
                            alias_confirmed.add(a[1])
                            break
                    old=dictionary[seed]        
                    dictionary[seed]=[min(old[0],x),max(old[1],x),min(old[2],y),max(old[3],y)]
        return [Seed(key,(round((value[1]+value[0])/2),round((value[3]+value[2])/2))) for key, value in dictionary.items() if key not in alias_confirmed]                   


    def region_growing(self, new_seeds):
        seeds=[Seed(label,point) for label,point in self.seed_dict.items()]
        
        im = self.matrix_to_img(self.data)
        otsu_threshold, thresh = cv2.threshold( im, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        max_value = max([max(i_data) for i_data in self.data])
        min_value = min([min(i_data) for i_data in self.data])
        threshold=(otsu_threshold*(max_value-min_value)/255)+min_value
        alias=set()
        seed_mark =  [[0 for _ in range(self.y_w)]
                     for _ in range(self.x_w)]

        seed_list=seeds
        while len(seed_list)>0:
            current=seed_list.pop(0)
            if self.data[current.point[0]][current.point[1]]<threshold:
                self.remove_seed(current.label)
                continue
            current_mark=seed_mark[current.point[0]][current.point[1]]
            if current_mark!=0 and current_mark!=current.label:
                alias1=max(current_mark,current.label)
                alias2=min(current_mark,current.label)
                alias.add((alias1,alias2))
            elif self.data[current.point[0]][current.point[1]]>threshold: # current_mark==0      
                seed_mark[current.point[0]][current.point[1]]=current.label
                for neighbour in self.neighbours(current):
                    if self.check(neighbour) and self.data[neighbour.point[0]][neighbour.point[1]]>threshold and seed_mark[neighbour.point[0]][neighbour.point[1]]==0:
                        seed_list.append(neighbour)

        # If there are high-probability areas that have not yet been assigned
        # to a pre-existing seed, add to the seeds the new observations that
        # lie in that region with the same algorithm used in the previous step
        # (May be a good idea to refactor this code)
        old_seeds = self.next_label-1
        cartesian_seed=[]
        for seed in new_seeds:
            point=self.__robot.odometry.polar_to_abs_cartesian(seed)
            coord=self.point_to_coord(point)
            if coord[0]<self.x_w and coord[1]<self.y_w \
                and self.data[coord[0]][coord[1]]>threshold \
                and (seed_mark[coord[0]][coord[1]] == 0 or seed_mark[coord[0]][coord[1]] > old_seeds): # Point not assigned to an old seed
                #print(seed_mark[coord[0]][coord[1]] > old_seeds)
                #print(self.coord_to_point((coord[0],coord[1])))
                cartesian_seed.append(Seed(self.next_label,coord))
                self.next_label+=1
                seed_list = [cartesian_seed[-1]]
                while len(seed_list) > 0:
                    current=seed_list.pop(0)
                    if self.data[current.point[0]][current.point[1]]<threshold:
                        self.remove_seed(current.label)
                        continue
                    current_mark=seed_mark[current.point[0]][current.point[1]]
                    if current_mark!=0 and current_mark!=current.label:
                        alias1=max(current_mark,current.label)
                        alias2=min(current_mark,current.label)
                        alias.add((alias1,alias2))
                    elif self.data[current.point[0]][current.point[1]]>threshold: # current_mark==0      
                        seed_mark[current.point[0]][current.point[1]]=current.label
                        for neighbour in self.neighbours(current):
                            if self.check(neighbour) and self.data[neighbour.point[0]][neighbour.point[1]]>threshold and seed_mark[neighbour.point[0]][neighbour.point[1]]==0:
                                seed_list.append(neighbour)
    
        self.seed_dict.update({seed.label: seed.point for seed in cartesian_seed})        
        return (seed_mark,alias)              


    def neighbours(self,point):
        l=[(-1, -1), (0, -1), (1, -1), (1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0)]
        return [Seed(point.label,(point.point[0]+x,point.point[1]+y)) for x,y in l]
        
    def check(self,point):
        return 0<=point.point[0]<self.x_w and 0<=point.point[1]<self.y_w


    def find_clusters_2(self):
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
        if self.show_map:
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

class Seed:
    def __init__(self,label,point):
        self.label=label
        self.point=point

    def __str__(self):
        return "Label: {} - Point: {}".format(self.label,self.point)    

