#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <signal.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include "ros/ros.h"

#include "ros_interface.hpp"

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/get_int.h>
#include <webots_ros/set_string.h>
#include <webots_ros/get_bool.h>
#include <webots_ros/get_float_array.h>
#include <webots_ros/speaker_play_sound.h>
#include <webots_ros/speaker_speak.h>
#include <webots_ros/display_image_load.h>
#include <webots_ros/display_image_paste.h>

ros::NodeHandle *n;
ros::ServiceClient time_step_client;
webots_ros::set_int time_step_srv;
// catch names of the controllers availables on ROS network
void controller_name_callback(const std_msgs::String::ConstPtr &name) ;



static int controller_count;
static std::vector<std::string> controller_list;
static double compassValues[3] = {0, 0, 0};

static const char *motor_names[N_MOTORS] = {"left_wheel_motor", "right_wheel_motor"};
static const char *motor_names_complete[N_MOTORS+1] = {"left_wheel_motor", "right_wheel_motor", "servo"};
const std::string name = "change";


// gaussian function
double gaussian(double x, double mu, double sigma) {
	return (1.0 / (sigma * sqrt(2.0 * M_PI))) * exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma));
}

void compassCallback(const sensor_msgs::MagneticField::ConstPtr &values) {
	compassValues[0] = values->magnetic_field.x;
	compassValues[1] = values->magnetic_field.y;
	compassValues[2] = values->magnetic_field.z;

	ROS_INFO("Compass values are x=%f y=%f z=%f (time: %d:%d).", compassValues[0], compassValues[1], compassValues[2],
			values->header.stamp.sec, values->header.stamp.nsec);	   
}

void distance_sensorCallback(const sensor_msgs::Range::ConstPtr &value) {
  	ROS_INFO("Distance from object is %f (time: %d:%d).", value->range, value->header.stamp.sec, value->header.stamp.nsec);
}


void get_device_values(const std::string device){
	ros::Subscriber sub;
	if ( !device.compare("compass") ){
		sub = n->subscribe(name + "/" + device + "/values", 1, compassCallback);
	} else if ( !device.compare("camera") ){
		exit(1);
	} else if ( !device.compare("gyro") ){
		exit(1);
	} else if ( !device.compare("accelerometer") ){
		exit(1);
	} else {
		exit(1);
	}
	while (sub.getNumPublishers() == 0) {
		ros::spinOnce();
		time_step_client.call(time_step_srv);
	}
	ros::spinOnce();
	time_step_client.call(time_step_srv);
	ros::spinOnce();

	sub.shutdown();
	time_step_client.call(time_step_srv);
}	

void enable_device(const std::string device){
	ros::ServiceClient set_device_client;
	webots_ros::set_int set_device_srv;
	set_device_client = n->serviceClient<webots_ros::set_int>(name+"/"+device+"/enable");
	set_device_srv.request.value = TIME_STEP;
	if(set_device_client.call(set_device_srv)){
		ROS_INFO("%s succefully enabled.", device.c_str());
	}else{
		ROS_ERROR("Failed to call service %s/enable.", device.c_str());
	}
	set_device_client.shutdown();
	time_step_client.call(time_step_srv);
}

void set_motor_position(const std::string motor, double position) {
	ros::ServiceClient set_position_client;
	webots_ros::set_float set_position_srv;
	set_position_client = n->serviceClient<webots_ros::set_float>(name + "/" + motor + "/set_position");
	set_position_srv.request.value = position;
	if(set_position_client.call(set_position_srv)){
		ROS_INFO("%s's position succefully setted.", motor.c_str());
	}else{
		ROS_ERROR("Failed to call service %s/set_position.", motor.c_str());
	}
	set_position_client.shutdown();
	time_step_client.call(time_step_srv);
}

void set_motor_speed(const std::string motor, double speed) {
	ros::ServiceClient set_velocity_client;
	webots_ros::set_float set_velocity_srv;
	set_velocity_client = n->serviceClient<webots_ros::set_float>(name + "/" + motor + "/set_velocity");
	set_velocity_srv.request.value = speed;
	if(set_velocity_client.call(set_velocity_srv)){
		ROS_INFO("%s's velocity succefully setted.", motor.c_str());
	}else{
		ROS_ERROR("Failed to call service %s/set_velocity.", motor.c_str());
	}
	set_velocity_client.shutdown();
	time_step_client.call(time_step_srv);
}

// Get true if speaker is speaking, false otherwise
bool is_speaking() {
	bool is_speaking=true;
	ros::ServiceClient speaker_is_speaking_client;
	webots_ros::get_bool speaker_is_speaking_srv;
	speaker_is_speaking_client = n->serviceClient<webots_ros::get_bool>(name + "/speaker/is_speaking");

	if (speaker_is_speaking_client.call(speaker_is_speaking_srv)) {
		is_speaking=speaker_is_speaking_srv.response.value;
	} else
		ROS_ERROR("Failed to call service is_speaking.");

	speaker_is_speaking_client.shutdown();
	time_step_client.call(time_step_srv);
	return is_speaking;
}

// Show on the display an image from filesystem
void image_load(const std::string imageName) {
	ros::ServiceClient display_image_load_client;
	webots_ros::display_image_load display_image_load_srv;
	display_image_load_client = n->serviceClient<webots_ros::display_image_load>(name + "/display/image_load");

	display_image_load_srv.request.filename = std::string("../../../../../Media/Image/")+ imageName +std::string(".jpg");
	display_image_load_client.call(display_image_load_srv);
	ROS_INFO("Image successfully loaded to clipboard.");
	uint64_t loaded_image = display_image_load_srv.response.ir;

	ros::ServiceClient display_image_paste_client;
	webots_ros::display_image_paste display_image_paste_srv;
	display_image_paste_client = n->serviceClient<webots_ros::display_image_paste>(name + "/display/image_paste");

	display_image_paste_srv.request.ir = loaded_image;
	if (display_image_paste_client.call(display_image_paste_srv) && display_image_paste_srv.response.success == 1)
		ROS_INFO("Image successfully load and paste.");
	else
		ROS_ERROR("Failed to call service display_image_paste to paste image.");

	display_image_load_client.shutdown();
	display_image_paste_client.shutdown();
	time_step_client.call(time_step_srv);
}

// TO FIX
// play a .mp3 from filesystem with volume between 0.0 and 1.0 
void play_sound(const std::string soundName, double volume, bool loop) {
	ros::ServiceClient speaker_play_sound_client;
	webots_ros::speaker_play_sound speaker_play_sound_srv;
	speaker_play_sound_client = n->serviceClient<webots_ros::speaker_play_sound>(name + "/speaker/play_sound");

	speaker_play_sound_srv.request.sound = std::string("../../../../../Media/Audio/")+ soundName +std::string(".mp3");
	speaker_play_sound_srv.request.volume = volume;
	speaker_play_sound_srv.request.pitch = 1.0;
	speaker_play_sound_srv.request.balance = 0.0;
	speaker_play_sound_srv.request.loop = loop;

	if (speaker_play_sound_client.call(speaker_play_sound_srv) && speaker_play_sound_srv.response.success == 1)
		ROS_INFO("Sound successfully load and played.");
	else
		ROS_ERROR("Failed to call service speaker_play_sound to play a sound.");

	speaker_play_sound_client.shutdown();
	time_step_client.call(time_step_srv);
}


// speak a text with volume between 0.0 and 1.0 
void speak(const std::string &text, double volume) {
	ros::ServiceClient speaker_speak_client;
	webots_ros::speaker_speak speaker_speak_srv;
	speaker_speak_client = n->serviceClient<webots_ros::speaker_speak>(name + "/speaker/speak");

	speaker_speak_srv.request.text = text;
	speaker_speak_srv.request.volume = volume;
	while (is_speaking()){}
	if (speaker_speak_client.call(speaker_speak_srv) && speaker_speak_srv.response.success == 1)
		ROS_INFO("Text successfully readed.");
	else
		ROS_ERROR("Failed to call service speaker_speak to read a text.");

	speaker_speak_client.shutdown();
	time_step_client.call(time_step_srv);
}


// Set speaker's language
// 0 for Italian.
// 1 for American English.
// 2 for German.
// 3 for Spanish.
// 4 for French.
// 5 for British English.
void set_language(const int language) {
	const std::string languages[N_LANGUAGES] = {"it-IT", "en-US", "de-DE", "es-ES", "fr-FR", "en-UK"};
	ros::ServiceClient speaker_set_language_client;
	webots_ros::set_string speaker_set_language_srv;
	speaker_set_language_client = n->serviceClient<webots_ros::set_string>(name + "/speaker/set_language");

	speaker_set_language_srv.request.value = languages[language];

	if (speaker_set_language_client.call(speaker_set_language_srv) && speaker_set_language_srv.response.success == 1)
		ROS_INFO("Language successfully setted.");
	else
		ROS_ERROR("Failed to call service set_language to set speaker's language.");

	speaker_set_language_client.shutdown();
	time_step_client.call(time_step_srv);
}

void speak_polyglot(const std::vector<std::string> text, double volume) {
	for (int i = 0; i < text.size(); ++i) {
		while (is_speaking()){}
		set_language(i);
		speak(text[i], volume);
	}  
}

// catch names of the controllers availables on ROS network
void controller_name_callback(const std_msgs::String::ConstPtr &name) {
	controller_count++;
	controller_list.push_back(name->data);
	ROS_INFO("Controller #%d: %s.", controller_count, controller_list.back().c_str());
}


void quit(int sig) {
	ROS_INFO("User stopped the 'change' node.");
	time_step_srv.request.value = 0;
	time_step_client.call(time_step_srv);
	ros::shutdown();
	exit(0);
}

void init(int argc, char **argv){
	std::string controllerName;
	// create a node named as "name" variable on ROS network
	ros::init(argc, argv, name, ros::init_options::AnonymousName);
	n = new ros::NodeHandle;

	signal(SIGINT, quit);

	// subscribe to the topic model_name to get the list of availables controllers
	ros::Subscriber nameSub = n->subscribe("model_name", 100, controller_name_callback);
	while (controller_count == 0 || controller_count < nameSub.getNumPublishers()) {
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
	}
	ros::spinOnce();

	time_step_client = n->serviceClient<webots_ros::set_int>(name+"/robot/time_step");
	time_step_srv.request.value = TIME_STEP;

	// if there is more than one controller available, it let the user choose
	if (controller_count == 1)
		controllerName = controller_list[0];
	else {
		int wantedController = 0;
		std::cout << "Choose the # of the controller you want to use:\n";
		std::cin >> wantedController;
		if (1 <= wantedController && wantedController <= controller_count)
			controllerName = controller_list[wantedController - 1];
		else {
			ROS_ERROR("Invalid number for controller choice.");
			exit(1);
		}
	}
	ROS_INFO("Using controller: '%s'", controllerName.c_str());
	// leave topic once it is not necessary anymore
	nameSub.shutdown();

	// init motors
	for (int i = 0; i < N_MOTORS+1; ++i) {
		set_motor_position(motor_names_complete[i],INFINITY);
		set_motor_speed(motor_names_complete[i],0.0);
	}

	// enable devices
	enable_device("gyro");
	enable_device("accelerometer");
	enable_device("camera");
	enable_device("compass");

	for (int i = 0; i < N_DISTANCE_SENSORS; ++i) {
		enable_device("ds"+std::to_string(i));
	}  


	ROS_INFO("You can now visualize the sensors output in rqt using 'rqt'.");

	// for test
	for (int i = 0; i < N_MOTORS+1; ++i) {
		set_motor_speed(motor_names_complete[i],MAX_SPEED/4);
	}

	get_device_values("compass");
	ROS_ERROR("%f %f %f", compassValues[0], compassValues[1], compassValues[2]);
	
	
	image_load("warning");
	

	set_language(0);
	speak("Ciao sono ciangà e sugnu troppu fuoitti",1.0);
	std::vector<std::string> ciao {"Ciao", "Hello", "Halo", "Hola", "Salut"};
	speak_polyglot(ciao, 1.0);

	/* TO FIX */
	//play_sound("warning", 1.0, 0);
}

int isOk(){
	return ros::ok();
}
void processCallbacks(){
	ros::spinOnce();
}
int timeStep(){
	if (!time_step_client.call(time_step_srv) ||
			!time_step_srv.response.success) {
		ROS_ERROR("Failed to call service time_step for next step.");
		return 0;
	}
	return 1;
}
