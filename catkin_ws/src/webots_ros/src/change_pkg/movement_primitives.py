from change_pkg.ros_interface import *
import cv2
import time

angular_velocity = 0.02
linear_velocity  = 5.0

def set_angular_velocity(speed):
    call_service(motors[0], 'set_velocity', -speed*330/2);
    call_service(motors[1], 'set_velocity',  speed*330/2);


def set_linear_velocity(speed):
    call_service(motors[0], 'set_velocity', speed);
    call_service(motors[1], 'set_velocity', speed);


def stop():
    set_linear_velocity(0)


def rotate(rotation, precision):
    """
    :rotation:  The desired rotation in degrees. Note that a rotation greater
        than 180° in modulo will be substituted with a rotation in the opposite
        direction.
    :precision: The     maximum difference from the required rotation and the
        actual rotation. If this value is smaller than the sensor noise it makes
        no sense.
    """
    stop()
    r = rospy.Rate(10) # 10hz
    curr_angular_velocity = angular_velocity
    framenum=1
    #processCallbacks()
    #update_frame()
    #update_object_roi()
    # adjust for discontinuity at +/-180°
    current_angle = 0
    difference = rotation-current_angle
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
        img = get_image()
        if(img.any() != None):
            cv2.imshow('feed',img)
            if(cv2.waitKey(1) == ' '):
                break

        gyro = get_gyro_values()
        time = gyro['t']
        if prev_time:
            ang_vel = 180*gyro['z']/math.pi
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
            set_angular_velocity(curr_angular_velocity*(-direction)\
                    * (min(1,abs(difference/45))) )
        prev_time = time
        prev_ang_vel = ang_vel
    stop()
    return current_angle


def move_forward(distance, precision):
    stop()
    distance_traveled=0
    speed=0
    prev_speed = 0
    prev_stamp = 0
    prev_accel = 0
    while(distance*1.1 - distance_traveled > precision):
        #img = get_image()
        #if(img.any() != None):
            #cv2.imshow('feed',img)
            #if(cv2.waitKey(1) == ' '):
                #break

        accel = get_accelerometer_values()
        timestamp = accel['t']
        accel = accel['x'] if abs(accel['x']) > 0.01 else 0
        if(prev_stamp):
            elapsed_time = timestamp-prev_stamp
            elapsed_time = elapsed_time.to_sec()
            prev_speed = speed
            speed = speed+((accel-prev_accel)/2+prev_accel)*elapsed_time
            distance_traveled = distance_traveled\
                    + (prev_speed+ (speed-prev_speed)/2 ) *elapsed_time
            set_linear_velocity(linear_velocity\
                    *(min(1, (distance*1.1-distance_traveled)/0.2) ) )
        prev_stamp = timestamp
        prev_accel = accel
    stop()
    return distance_traveled
