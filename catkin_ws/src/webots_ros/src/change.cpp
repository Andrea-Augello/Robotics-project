#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <signal.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include "ros/ros.h"


#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_string.h>
#include <webots_ros/get_bool.h>
#include <webots_ros/speaker_play_sound.h>
#include <webots_ros/speaker_speak.h>
#include <webots_ros/display_image_load.h>
#include <webots_ros/display_image_paste.h>

#include "ros_interface.hpp"

int main(int argc, char **argv) {
	init(argc, argv);
	// main loop
	while (ros::ok()) {
		if (!time_step_client.call(time_step_srv) ||
				!time_step_srv.response.success) {
			ROS_ERROR("Failed to call service time_step for next step.");
			break;
		}
		ros::spinOnce();
	}
	time_step_srv.request.value = 0;
	time_step_client.call(time_step_srv);

	ros::shutdown();
	return 0;
}
