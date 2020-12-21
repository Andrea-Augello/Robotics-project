from std_msgs.msg import *
from sensor_msgs.msg import *
import cv2
import rospy
import time
import math

class Movement:
    def __init__(self, robot):
        self.robot=robot
        self.angular_velocity = 0.02
        self.linear_velocity  = 5.0
        

    def set_angular_velocity(self,angular_velocity):
        self.robot.motors.left_wheel.set_velocity(-angular_velocity*330/2)
        self.robot.motors.right_wheel.set_velocity(angular_velocity*330/2)


    def set_linear_velocity(self,linear_velocity):
        self.robot.motors.left_wheel.set_velocity(linear_velocity)
        self.robot.motors.right_wheel.set_velocity(linear_velocity)



    def stop(self):
        self.set_linear_velocity(0)


    def rotate(self,rotation, precision=0.01):
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

            gyro=self.robot.sensors.gyro.value
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
                self.set_angular_velocity(curr_angular_velocity*(-direction)\
                        * (min(1,abs(difference/45))) )
            prev_time = time
            prev_ang_vel = ang_vel
        self.stop()
        self.robot.odometry.update_theta(current_angle)
        return current_angle


    def move_forward(self, distance, precision=0.01):
        """
        :returns: traveled distance as a vector

        """
        self.stop()
        diameter=self.robot.wheel_diameter
        distance_traveled=[0,0]
        speed=[0,0]
        prev_speed = [0,0]
        prev_stamp = 0
        prev_accel = [0,0]

        rotations = distance/(math.pi*diameter)
        angle = rotations * 2 * math.pi
        left_wheel_target = self.robot.sensors.left_wheel.value + angle
        right_wheel_target = self.robot.sensors.right_wheel.value + angle
        precision = precision*2/diameter

        self.robot.motors.left_wheel.set_position(left_wheel_target)
        self.robot.motors.right_wheel.set_position(right_wheel_target)

        self.set_linear_velocity(self.linear_velocity)

        while(abs(right_wheel_target-self.robot.sensors.left_wheel.value)>precision \
                and abs(left_wheel_target-self.robot.sensors.left_wheel.value)>precision):
            accel = self.robot.sensors.accelerometer.value
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
        self.robot.motors.left_wheel.init()
        self.robot.motors.right_wheel.init()
        self.robot.odometry.update_position(distance_traveled)
        return distance_traveled

        


    
    def move_forward_accel(self, distance, precision=0.01):
        self.stop()
        distance_traveled=0
        speed=0
        prev_speed = 0
        prev_stamp = 0
        prev_accel = 0
        coeff = 1.1
        while(distance*coeff - distance_traveled > precision):

            accel = self.robot.sensors.accelerometer.value
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
        self.robot.odometry.update_position(distance_traveled)
        return distance_traveled
    
    def scan(self):
        self.robot.vision.clear_saved_frames()
        rotation = 0
        for _ in range(7):
            rotation = rotation + self.rotate(self.robot.vision.HORIZONTAL_FOV,0.1)
            self.robot.vision.save_frame(self.robot.vision.update_frame(self.robot.sensors.camera.value))
        offset = rotation % 360
        rotation = rotation + self.rotate(-offset,0.1)
        return rotation  

    

