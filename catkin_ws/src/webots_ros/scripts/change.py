#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from webots_ros.srv import *
import os

model_name = 'change'
time_step = 32


def callback(data):
    global velocity
    global message
    message = 'Received velocity value: ' + str(data.data)
    velocity = data.data

def motor_set_position(motor,position):
	service = model_name+"/"+motor+ "/set_position"
	rospy.wait_for_service(service)
	try:
		motor_set_position_service = rospy.ServiceProxy(service,_set_float)
		resp = motor_set_position_service(position)
		return resp
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)	

def motor_set_velocity(motor,velocity):
	service = model_name+"/"+motor+ "/set_velocity"
	rospy.wait_for_service(service)
	try:
		motor_set_velocity_service = rospy.ServiceProxy(service,_set_float)
		resp = motor_set_velocity_service(velocity)
		return resp
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)			

def main():
	while not rospy.is_shutdown():
		rospy.loginfo('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
		rospy.loginfo('Time step: ' + str(time_step))
		motor_set_position('left_wheel_motor',float('inf'))
		motor_set_velocity('left_wheel_motor',5.0)
		rospy.init_node(model_name, anonymous=True)

		

def main2():
	robot = Robot()
	timeStep = int(robot.getBasicTimeStep())
	left = robot.getMotor('motor.left')
	right = robot.getMotor('motor.right')
	sensor = robot.getDistanceSensor('prox.horizontal.2')  # front central proximity sensor
	sensor.enable(timeStep)
	left.setPosition(float('inf'))  # turn on velocity control for both motors
	right.setPosition(float('inf'))
	velocity = 0
	left.setVelocity(velocity)
	right.setVelocity(velocity)
	message = ''
	print('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
	robot.step(timeStep)
	rospy.init_node('listener', anonymous=True)
	print('Subscribing to "motor" topic')
	robot.step(timeStep)
	rospy.Subscriber('motor', Float64, callback)
	pub = rospy.Publisher('sensor', Float64, queue_size=10)
	print('Running the control loop')
	while robot.step(timeStep) != -1 and not rospy.is_shutdown():
		pub.publish(sensor.getValue())
		print('Published sensor value: ', sensor.getValue())
		if message:
			print(message)
			message = ''
		left.setVelocity(velocity)
		right.setVelocity(velocity)


if __name__ == "__main__":
	main()    

