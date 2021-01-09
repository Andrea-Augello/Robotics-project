from std_msgs.msg import *
from sensor_msgs.msg import *
import cv2
import math

class Movement:
    def __init__(self, robot):
        self.__robot=robot
        self.angular_velocity = 0.02
        self.linear_velocity  = 5.0
        self.SECURITY_DISTANCE = 0.75
        self.SPEED_THRESHOLD = 0.1
        self.ACCELERATION_THRESHOLD = 0.01

    def set_angular_velocity(self,angular_velocity):
        self.__robot.motors.left_wheel.set_velocity(-angular_velocity*330/2)
        self.__robot.motors.right_wheel.set_velocity(angular_velocity*330/2)


    def set_linear_velocity(self,linear_velocity):
        self.__robot.motors.right_wheel.set_velocity(linear_velocity)
        self.__robot.motors.left_wheel.set_velocity(linear_velocity)



    def stop(self, linear=False, speed=[0,0], prev_accel=[0,0], prev_stamp=0):
        self.set_linear_velocity(0)
        if linear:
            prev_speed = speed
            distance_traveled = [0,0]
            while( speed[0]>self.SPEED_THRESHOLD or speed[1]>self.SPEED_THRESHOLD):
                accel = self.__robot.sensors.accelerometer.value
                timestamp = accel.t
                accel = [accel.x if abs(accel.x) > self.ACCELERATION_THRESHOLD else 0,accel.y if abs(accel.y) > self.ACCELERATION_THRESHOLD else 0]
                if(prev_stamp):
                    elapsed_time = timestamp-prev_stamp
                    elapsed_time = elapsed_time.to_sec()
                    prev_speed = speed
                    for i in range(2):
                        speed[i] += ((accel[i]+prev_accel[i])/2)*elapsed_time
                prev_stamp = timestamp
                prev_accel = accel
        else:
            while abs(self.__robot.sensors.gyro.value.z) > 0.01:
                pass
            

    def rotate(self,rotation, precision=1):
        """
        :rotation:  The desired rotation in degrees. Note that a rotation greater
            than 180° in modulo will be substituted with a rotation in the opposite
            direction.
        :precision: The maximum difference from the required rotation and the
            actual rotation. If this value is smaller than the sensor noise it makes
            no sense.
        """
        self.stop()
        if(abs(rotation) <= precision):
            return 0
        starting_angle = self.__robot.odometry.theta
        current_angle = starting_angle
        target_angle=rotation+current_angle-math.copysign(precision,rotation)
        curr_angular_velocity = self.angular_velocity
        # adjust for discontinuity at +/-180°
        difference = rotation
        current_angle = 0
        if(difference > 180):
            difference = difference - 360
        elif (difference < -180):
            difference = difference + 360

        direction =  1 if difference > 0 else -1
        while(abs(difference)>precision ):
        # update_frame()
            current_angle = self.__robot.odometry.theta
            difference = target_angle-current_angle
            if(difference > 180):
                difference = difference - 360
            elif (difference < -180):
                difference = difference + 360
            
            if(direction*difference < 0):
            # if we went over the specified angle, reverses the direction and
            # decreases the angular speed to have a better chance of sampling
            # at the right moment.
                curr_angular_velocity*=0.8
                direction=-direction
            
            # applies rudimentary proportional controller
            self.set_angular_velocity(curr_angular_velocity*(-direction)\
                    * (min(1,abs(difference/45))) )
        self.stop(linear=False)
        return self.__robot.odometry.theta - starting_angle


    def move_forward(self, distance, precision=0.01):
        """
        :returns: traveled distance as a vector

        """
        self.stop(linear=True)
        diameter=self.__robot.wheel_diameter
        distance_traveled=[0,0]
        speed=[0,0]
        timestamp=0
        prev_speed = [0,0]
        prev_stamp = 0
        prev_accel = [0,0]
        timestamp=0
        rotations = distance/(math.pi*diameter)
        angle = rotations * 2 * math.pi
        left_wheel_target = self.__robot.sensors.left_wheel.value + angle
        right_wheel_target = self.__robot.sensors.right_wheel.value + angle
        precision = precision*2/diameter

        self.__robot.motors.left_wheel.set_position(left_wheel_target)
        self.__robot.motors.right_wheel.set_position(right_wheel_target)

        self.set_linear_velocity(self.linear_velocity)
        lidar_position=math.floor(len(self.__robot.sensors.lidar.value)/2)
        while(abs(right_wheel_target-self.__robot.sensors.right_wheel.value)>precision\
                and abs(left_wheel_target-self.__robot.sensors.left_wheel.value)>precision\
                and min([self.__robot.sensors.lidar.value[lidar_position-4][0],
                    self.__robot.sensors.lidar.value[lidar_position-3][0],
                    self.__robot.sensors.lidar.value[lidar_position+2][0],
                    self.__robot.sensors.lidar.value[lidar_position+3][0]])>self.SECURITY_DISTANCE*0.66 \
                and min([self.__robot.sensors.lidar.value[lidar_position-6][0],
                    self.__robot.sensors.lidar.value[lidar_position-5][0],
                    self.__robot.sensors.lidar.value[lidar_position+4][0],
                    self.__robot.sensors.lidar.value[lidar_position+5][0]])>self.SECURITY_DISTANCE*0.33 \
                and min([self.__robot.sensors.lidar.value[lidar_position-2][0],
                    self.__robot.sensors.lidar.value[lidar_position-1][0],
                    self.__robot.sensors.lidar.value[lidar_position][0],
                    self.__robot.sensors.lidar.value[lidar_position+1][0]])>self.SECURITY_DISTANCE \
                and not self.__robot.sensors.bumper.value):
            accel = self.__robot.sensors.accelerometer.value
            timestamp = accel.t
            accel = [accel.x, accel.y]# [accel.x if abs(accel.x) > 0.01 else 0,accel.y if abs(accel.y) > 0.01 else 0]
            if(prev_stamp):
                elapsed_time = timestamp-prev_stamp
                elapsed_time = elapsed_time.to_sec()
                prev_speed = speed
                for i in range(2):
                    speed[i] +=((accel[i]+prev_accel[i])/2)*elapsed_time
            prev_stamp = timestamp
            prev_accel = accel
        self.stop(linear=True, speed=speed, prev_stamp=timestamp, prev_accel=prev_accel)
        self.__robot.motors.left_wheel.init()
        self.__robot.motors.right_wheel.init()
        self.__robot.odometry.update_position()

        


    
    def move_forward_accel(self, distance, precision=0.01):
        self.stop()
        distance_traveled=0
        speed=0
        prev_speed = 0
        prev_stamp = 0
        prev_accel = 0
        coeff = 1.1
        while(distance*coeff - distance_traveled > precision):

            accel = self.__robot.sensors.accelerometer.value
            timestamp = accel.t
            accel = accel.x if abs(accel.x) > 0.01 else 0
            if(prev_stamp):
                elapsed_time = timestamp-prev_stamp
                elapsed_time = elapsed_time.to_sec()
                prev_speed = speed
                speed = speed+((accel-prev_accel)/2+prev_accel)*elapsed_time
                distance_traveled = distance_traveled\
                        + (prev_speed+ (speed-prev_speed)/2 ) *elapsed_time
                self.set_linear_velocity(self.linear_velocity\
                        *(min(1, (distance*coeff-distance_traveled)/0.2) ) )
            prev_stamp = timestamp
            prev_accel = accel
        self.stop()
        self.__robot.odometry.update_position(distance_traveled)
        return distance_traveled
    
    def scan(self):
        for _ in range(2):
            self.__robot.tablet.warning()
        self.__robot.vision.clear_saved_frames()
        rotation = 0
        self.__robot.vision.save_frame(self.__robot.vision.update_frame(self.__robot.sensors.camera.value))
        for i in range(math.ceil(360/self.__robot.vision.HORIZONTAL_FOV)-1):
            rotation = rotation + self.rotate(self.__robot.vision.HORIZONTAL_FOV,precision=1)
            error = (rotation - (i+1)*self.__robot.vision.HORIZONTAL_FOV)%360
            error = error if error < 180 else error-360
            self.__robot.set_pose(error*math.pi/180,0)
            self.__robot.vision.save_frame(self.__robot.vision.update_frame(self.__robot.sensors.camera.value))
        offset = rotation % 360
        offset = offset if offset < 180 else offset-360
        self.__robot.set_pose(0,0)
        rotation = rotation + self.rotate(-offset,0.1)
        return rotation  
