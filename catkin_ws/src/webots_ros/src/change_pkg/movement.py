from change_pkg.vision import *
from std_msgs.msg import *
from sensor_msgs.msg import *
import cv2
import time
import math

robot = None

class Movement:
    def __init__(self, r):
        global robot
        robot=r
        self.static_angular_velocity = 0.02
        self.static_linear_velocity  = 5.0
        

    @property
    def angular_velocity(self):
        return self.__angular_velocity   

    @angular_velocity.setter
    def angular_velocity(self,angular_velocity):
        global robot
        robot.motors.left_wheel.velocity=-angular_velocity*330/2
        robot.motors.right_wheel.velocity=angular_velocity*330/2
        self.__angular_velocity=angular_velocity

    @property
    def linear_velocity(self):
        return self.__linear_velocity

    @linear_velocity.setter
    def linear_velocity(self,linear_velocity):
        global robot
        robot.motors.left_wheel.velocity=linear_velocity
        robot.motors.right_wheel.velocity=linear_velocity
        self.__linear_velocity=linear_velocity


    def stop(self):
        self.linear_velocity=0


    def rotate(self,rotation, precision):
        """
        :rotation:  The desired rotation in degrees. Note that a rotation greater
            than 180° in modulo will be substituted with a rotation in the opposite
            direction.
        :precision: The     maximum difference from the required rotation and the
            actual rotation. If this value is smaller than the sensor noise it makes
            no sense.
        """
        self.stop()
        curr_angular_velocity = self.angular_velocity
        framenum=1
        #update_object_roi()
        # adjust for discontinuity at +/-180°
        difference = rotation
        current_angle = 0
        ang_vel = 0
        prev_ang_vel = 0
        prev_time = 0
        if(difference > 180):
            difference = difference - 360
        elif (difference < -180):
            difference = difference + 360

        direction =  1 if difference > 0 else -1
        while(abs(difference)>precision):
            # just for funzies
        # update_frame()
            if(cv2.waitKey(1) == ord(' ')):
                break

            gyro=robot.sensors.gyro.value
            time = gyro.t
            if prev_time:
                ang_vel = 180*gyro.z/math.pi
                elapsed_time = time - prev_time
                elapsed_time = elapsed_time.to_sec()
                current_angle = current_angle \
                        - (prev_ang_vel + (ang_vel-prev_ang_vel)/2 ) *elapsed_time
                difference = rotation-current_angle
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
                self.angular_velocity(curr_angular_velocity*(-direction)\
                        * (min(1,abs(difference/45))) )
            prev_time = time
            prev_ang_vel = ang_vel
        self.stop()
        return current_angle


    def move_forward(self, distance, precision):
        """
        :returns: traveled distance as a vector

        """
        self.stop()

        distance_traveled=[0,0]
        speed=[0,0]
        prev_speed = [0,0]
        prev_stamp = 0
        prev_accel = [0,0]

        rotations = distance/(math.pi*0.24)
        angle = rotations * 2 * math.pi
        left_wheel_target = robot.sensors.left_wheel.value + angle
        right_wheel_target = robot.sensors.right_wheel.value + angle
        precision = precision*2/0.24

        robot.motors.left_wheel.position=left_wheel_target
        robot.motors.right_wheel.position=right_wheel_target

        self.linear_velocity=self.static_linear_velocity


        while(abs(right_wheel_target-robot.sensors.left_wheel.value)>precision \
                and abs(left_wheel_target-robot.sensors.left_wheel.value)>precision):
            accel = robot.sensors.accelerometer.value
            timestamp = accel.t
            accel = [accel.x if abs(accel.x) > 0.01 else 0,accel.y if abs(accel.y) > 0.01 else 0]
            if(prev_stamp):
                elapsed_time = timestamp-prev_stamp
                elapsed_time = elapsed_time.to_sec()
                prev_speed = speed
                for i in range(2):
                    speed[i] = speed[i]+((accel[i]-prev_accel[i])/2+prev_accel[i])*elapsed_time
                    distance_traveled[i] = distance_traveled[i]\
                            + (prev_speed[i]+ (speed[i]-prev_speed[i])/2 ) *elapsed_time
            prev_stamp = timestamp
            prev_accel = accel
        robot.motors.left_wheel.init()
        robot.motors.right_wheel.init()
        return distance_traveled

        


    
    def move_forward_accel(self, distance, precision):
        self.stop()
        distance_traveled=0
        speed=0
        prev_speed = 0
        prev_stamp = 0
        prev_accel = 0
        while(distance - distance_traveled > precision):

            accel = robot.sensors.accelerometer.value
            timestamp = accel.t
            accel = accel.x if abs(accel.x) > 0.01 else 0
            if(prev_stamp):
                elapsed_time = timestamp-prev_stamp
                elapsed_time = elapsed_time.to_sec()
                prev_speed = speed
                speed = speed+((accel-prev_accel)/2+prev_accel)*elapsed_time
                distance_traveled = distance_traveled\
                        + (prev_speed+ (speed-prev_speed)/2 ) *elapsed_time
                self.linear_velocity=linear_velocity\
                        *(min(1, (distance*1.1-distance_traveled)/0.2) ) 
            prev_stamp = timestamp
            prev_accel = accel
        self.stop()
        return distance_traveled
    

    def scan():
        clear_saved_frames()
        rotation = 0
        for _ in range(7):
            rotation = rotation + rotate(57.29578,0.1)
            save_frame(update_frame())
        offset = rotation % 360
        rotation = rotation + rotate(-offset,0.1)
        return rotation

