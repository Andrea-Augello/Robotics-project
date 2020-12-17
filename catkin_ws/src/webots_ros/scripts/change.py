import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from webots_ros.msg import *
import os
import rosservice

model_name = 'change'
time_step = 32
sensors = {"Hokuyo_URG_04LX_UG01":True,
        "accelerometer":True,
        "base_cover_link":True, 
        "base_sonar_01_link":True,
        "base_sonar_02_link":True,
        "base_sonar_03_link":True, 
        "battery_sensor":False,
        "camera":True,
        "compass":True,
        "gyro":True, 	
        "head_1_joint_sensor":False,
        "head_2_joint_sensor":False, 
        "inertial_unit":False,
        "joystick":False,
        "keyboard":False,
        "torso_lift_joint_sensor":False,
        "wheel_left_joint_sensor":False,
        "wheel_right_joint_sensor":False
        }
motors = ["wheel_left_joint", "wheel_right_joint", "head_1_joint", "head_2_joint", "torso_lift_joint"]
compass_values = {'x':0, 'y':0, 'z':0}


def call_service(device_name,service_name,*args):
	service_string = "/%s/%s/%s" % (model_name, device_name, service_name)
	rospy.loginfo(service_string)
	rospy.wait_for_service(service_string)
	try:
		service = rospy.ServiceProxy(service_string,rosservice.get_service_class_by_name(service_string))
		response = service(*args)
		rospy.loginfo("Service %s called" % service_string)
		return response
	except rospy.ServiceException as e:
		rospy.logerr("Service call failed: %s" % e)

def enable_sensors():
	for sensor, flag in sensors.items():
		if(flag):
			call_service(sensor,"enable",time_step)

def motor_init():
	for motor in motors:
		call_service(motor,'set_position',float('inf'))
		call_service(motor,'set_velocity',0.0)

def load_image(image):
	image_loaded = call_service('display','image_load','../../../../../Media/Image/%s.jpg' % image)
	call_service('display','image_paste',image_loaded.ir,0,0,False)

#Don't work
def play_sound(sound):
	call_service('speaker','play_sound','../../../../../Media/Audio/%s.mp3' % sound, 1.0, 1.0, 0.0, False)

def is_speaking():
	response = call_service('speaker','is_speaking')
	return response.value

def speak(text):
	while(is_speaking()):
		pass
	call_service('speaker','speak', text, 1.0)


def speak_polyglot(it_IT=None,en_US=None,de_DE=None,es_ES=None,fr_FR=None,en_UK=None):
	for language, text in locals().items():
		if text is not None:
			call_service('speaker', 'set_language', language.replace("_","-"))
			speak(text)


def compassCallback(values):
	global compass_values
	compass_values['x'] = values.magnetic_field.x
	compass_values['y'] = values.magnetic_field.y
	compass_values['z'] = values.magnetic_field.z


def get_compass_values(sensor_name):
	service_string = "/%s/%s/values" % (model_name, sensor_name)
	return rospy.Subscriber(service_string, MagneticField, eval("compassCallback"))


def get_sensor_values():
	get_compass_values('compass')				
	

  

from ros_interface import *
import cv2

angular_velocity = 3.0
linear_velocity  = 3.0

def set_angular_velocity(speed):
    call_service(motors[0], 'set_velocity',-speed*330/2);
    call_service(motors[1], 'set_velocity', speed*330/2);


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
    curr_angular_velocity = angular_velocity
    direction, framenum=1
    processCallbacks()
    #update_frame()
    #update_object_roi()
    current_angle = get_angle()
    target_angle=rotation+get_angle()
    # adjust for discontinuity at +/-180°
    target_angle =  target_angle-360 if target_angle >  180 else target_angle
    target_angle =  360+target_angle if target_angle < -180 else target_angle
    difference = target_angle-current_angle
    if(difference > 180):
        difference = difference - 360
    elif (difference < -180):
        difference = difference + 360

    direction =  1 if difference > 0 else -1
    while(abs(difference)>precision):
        #update_frame()
        framenum = (framenum+1)%100
        if(framenum == 0):
            if(cv2.waitKey(1) == ' '):
                break

        current_angle = get_angle()
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

        set_angular_velocity(curr_angular_velocity*(-direction))
    stop()

#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from webots_ros.srv import *
from movement_primitives import *
from ros_interface import *
import os
import rosservice



def testing():
    #set_linear_velocity(3.0)
    load_image('warning')
    call_service('speaker', 'set_language', 'it-IT')
    speak("Ciao sono ciangà e sugnu troppu fuoitti")
    speak_polyglot(it_IT="ciao", en_UK="Hello")
    rospy.logerr(rospy.get_published_topics(namespace='/%s'%model_name))
	

def main():
    if not rospy.is_shutdown():
        rospy.init_node(model_name, anonymous=True)
        rospy.loginfo('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
        rospy.loginfo('Time step: ' + str(time_step))
        motor_init()
        enable_sensors()
        get_sensor_values()
        testing()
        rospy.spin()

                

if __name__ == "__main__":  
    main()    

