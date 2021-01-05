import math
from change_pkg.vision import *
import change_pkg.utils as utils
import numpy as np
import matplotlib.path as mpltPath

class Path_planner:

    """Docstring for Path_planner. """

    def __init__(self, robot, resolution=1, radius=5):
        """Accepts a robot object

        :robot: The robot this module belongs to

        """
        self.__robot = robot
        self.valid_target = False
        self.first_scan = True
        self.target = (self.__robot.odometry.x,self.__robot.odometry.y)
        self.distance_allowed = 0
        self.perimeter_target = []
        self.area_target = 0
        self.resolution = resolution
        self.radius=radius
        self.KP = 5
        self.ETA = 25
        self.OSCILLATIONS_DETECTION_LENGTH = 3
        self.SECURITY_DISTANCE = 2.0
        self.MAX_DEVIATION = 45
        self.MIN_EXPLORATION_STEP = 1
        self.LIDAR_THRESHOLD = 0.15 # Percentage of lidar FOV to discard, this
        # prevents steering to avoid objects that do not represent an obstacle
        # if the current trajectory is mantained


    def set_target(self, targets):
        """Sets the target point for the movement algorithm

        :targets: Dict of targets' coordinate
        :returns: None

        """
        target= None # targets[0]
        for t in targets:
            if not self.valid_target and (
                    mpltPath.Path(self.perimeter_target).contains_points([t['center']])\
                            or mpltPath.Path(t['contour']).contains_points([self.target])):
                continue
            else:
                if target == None or not self.first_scan and (t['area']>max(1.1304,target['area']) \
                    and ((mpltPath.Path(self.perimeter_target).contains_points([t['center']])\
                            or mpltPath.Path(t['contour']).contains_points([self.target]))\
                            or not mpltPath.Path(self.perimeter_target).contains_points([target['center']])\
                            or not mpltPath.Path(target['contour']).contains_points([self.target]))):
                    target=t
        if target == None:
            return False
        else:
            self.first_scan = False

        self.target = target['center']
        self.area_target = target['area']
        self.perimeter_target = target['contour']

        min_point = self.perimeter_target[0]
        min_distance = utils.math_distance(min_point,self.target)
        for perimeter_point in self.perimeter_target:
            perimeter_point_distance=utils.math_distance(perimeter_point,self.target)
            if  perimeter_point_distance < min_distance:
                #min_point = perimeter_point
                min_distance = perimeter_point_distance
        self.distance_allowed = min_distance
        self.valid_target=True
        return True


    def movement_distance(self):
        max_distance = self.max_distance()
        # rho = 0.5 + (MAX - 0.5)/(1 + (theta/30))
        max_distance_allowed= max_distance if max_distance < 0.5\
                else 0.5 + ( max_distance - 0.5)/(1 + (abs(self.target_angle())/30))
        distance = min(max_distance_allowed,self.target_distance())
        return distance

    def max_distance(self):
        lidar_position=math.floor(len(self.__robot.sensors.lidar.value)/2)
        # Selects the lidar slices corresponding to
        # the 40° fov in front of the robot
        max_distance =min(  # Max traveleable distance before obstacle
                self.__robot.sensors.lidar.value[lidar_position][0],
                self.__robot.sensors.lidar.value[lidar_position+1][0],
                self.__robot.sensors.lidar.value[lidar_position-1][0],
                self.__robot.sensors.lidar.value[lidar_position-2][0])
        return max_distance*0.9          

    def is_obstacle(self, distance):
        return distance < self.SECURITY_DISTANCE   


    def exploration_next_step(self):
        distance=self.max_distance()
        if distance > self.MIN_EXPLORATION_STEP:
            return (distance,0)
        else:
            direction = -1 if self.__robot.sensors.lidar.value[0][0] > self.__robot.sensors.lidar.value[-1][0] else 1
            return (0,direction*90)


    def bug_next_step(self):
        """ 
        Using the tangent bug method computes the movement direction for
        the next step towards the target and the movement distance.
        Only the tangent following part of the algorithm is implemented.

        :returns: (distance, angle)
        """
        target_angle = self.__robot.odometry.abs_cartesian_to_polar(self.target)[1]       
        direction = target_angle
        size=len(self.__robot.sensors.lidar.value)

        inf_limit=2
        sup_limit=size-inf_limit
        obstacles = self.__robot.sensors.lidar.value[inf_limit:sup_limit]
        width = 1.3*self.__robot.footprint 

        tangents=[]
        corrections=[]
        for i in range(len(obstacles)-1):
            delta = obstacles[i][0] - obstacles[i+1][0]
            if(delta > width\
                    or ( obstacles[i][0]>=self.__robot.sensors.lidar.range_max \
                    and obstacles[i+1][0]<self.__robot.sensors.lidar.range_max)):
               tangents.append((obstacles[i+1][0],obstacles[i][1]))
               corrections.append(-math.asin(min(1,width/obstacles[i+1][0])))
            elif ( delta < -width \
                    or ( obstacles[i][0] <self.__robot.sensors.lidar.range_max \
                    and obstacles[i+1][0]>=self.__robot.sensors.lidar.range_max)):
               tangents.append((obstacles[i][0],obstacles[i+1][1]))
               corrections.append(math.asin(min(1,width/obstacles[i][0])))

        found_tan = None
        tangent_distance = None
        for i in range(len(tangents)):
            new_tangent_distance = (target_angle-tangents[i][1])%360
            new_tangent_distance =\
                    new_tangent_distance if new_tangent_distance < 180 \
                    else abs(new_tangent_distance -360)
            if found_tan == None:
                found_tan = i
                tangent_distance = new_tangent_distance
            else:
                if (tangent_distance > new_tangent_distance ):
                    tangent_distance = new_tangent_distance
                    found_tan = i
        if found_tan != None :
            return (max(2,tangents[found_tan][0]),
                    tangents[found_tan][1]+corrections[found_tan]*180/math.pi)
        else:
            # Turn back and hope for the best
            return (0,180)


    def next_step_direction(self):
        """ 
        Using the potential fields method computes the movement direction for
        the next step towards the target 

        :returns: rotation angle
        """
        target_angle = self.__robot.odometry.abs_cartesian_to_polar(self.target)[1]       
        direction = target_angle
        size=len(self.__robot.sensors.lidar.value)
        inf_limit=math.ceil(size*self.LIDAR_THRESHOLD)
        sup_limit=size-inf_limit
        obstacles = [ p for p in self.__robot.sensors.lidar.value[inf_limit:sup_limit] if self.is_obstacle(p[0]) ]
        if len(obstacles):
            attractive_potential = self.KP * np.hypot(
                    self.__robot.odometry.x - self.target[0],
                    self.__robot.odometry.y - self.target[1])
            repulsive_potential = [0,0]
            for obstacle in obstacles:
                repulsive_potential_module = self.ETA \
                        * (1/obstacle[0]**2-1/self.SECURITY_DISTANCE**2 )
                repulsive_potential[0] = repulsive_potential_module * math.sin(obstacle[1]/180*math.pi)
                repulsive_potential[1] = repulsive_potential_module * math.cos(obstacle[1]/180*math.pi)
            direction = 180/math.pi*math.atan2( 
                    attractive_potential*math.sin(target_angle/180*math.pi) - repulsive_potential[0],
                    attractive_potential*math.cos(target_angle/180*math.pi) - repulsive_potential[1] )
            direction = max(-self.MAX_DEVIATION, min(self.MAX_DEVIATION, direction))
        return direction

    def target_distance(self):
        dist, angle = self.__robot.odometry.abs_cartesian_to_polar(self.target)
        return dist

    def target_angle(self):
        dist, angle = self.__robot.odometry.abs_cartesian_to_polar(self.target)
        return angle    
