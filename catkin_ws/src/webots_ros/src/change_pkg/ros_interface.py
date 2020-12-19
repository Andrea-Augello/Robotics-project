import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from webots_ros.msg import *
import os
import math
import rosservice
from scipy.spatial.transform import Rotation

model_name = 'change'
time_step = 32
sensors = {"Hokuyo_URG_04LX_UG01":True,
        "accelerometer":True,
        "base_cover_link":False, 
        "base_sonar_01_link":False,
        "base_sonar_02_link":False,
        "base_sonar_03_link":False, 
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
yaw = 0
compass_value = 0
gyro_values = {'x':0, 'y':0, 'z':0}
accelerometer_values = {'x':0, 'y':0, 'z':0, 'timestamp':0}


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


def Hokuyo_URG_04LX_UG01_callback(values):
    pass

def accelerometer_callback(values):
    global accelerometer_values
    accelerometer_values['x'] = values.linear_acceleration.x
    accelerometer_values['y'] = values.linear_acceleration.y
    accelerometer_values['z'] = values.linear_acceleration.z
    accelerometer_values['timestamp'] = values.header.stamp

def camera_callback(values):
	pass

def gyro_callback(values):
    global gyro_values
    gyro_values['x']=values.angular_velocity.x
    gyro_values['y']=values.angular_velocity.y
    gyro_values['z']=values.angular_velocity.z


def compass_callback(values):
    global compass_value 
    compass_value = 180*math.atan2(values.magnetic_field.x, values.magnetic_field.z)/math.pi


def inertial_unit_callback(values):
    global yaw       
    q = values.orientation
    rot = Rotation.from_quat([q.x,q.y,q.z,q.w])
    rot_euler = rot.as_euler('xyz', degrees=True) 
    yaw = 180*rot_euler[2]/math.pi


def get_sensor_value(topic, device, msg_type):
    try:
        return rospy.Subscriber(topic, msg_type, eval("%s_callback"%device))
    except NameError as e:
        rospy.logerr(str(e))
    


def get_sensors_values():
    for sensor in rospy.get_published_topics(namespace='/%s'%model_name):
        if 'range_image' not in sensor[0]: 
            msg_type=globals()[sensor[1].split("/")[1]]
            topic=sensor[0]
            device=sensor[0].split("/")[2]
            get_sensor_value(topic, device, msg_type)
    
                                        
def get_angle():
    global compass_value
    return compass_value

def get_accelerometer_values():
    global accelerometer_values
    return accelerometer_values

def get_gyro_values():
    global gyro_values
    return gyro_values         

  

