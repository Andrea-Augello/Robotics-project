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
#include <webots_ros/speaker_play_sound.h>
#include <webots_ros/speaker_speak.h>
#include <webots_ros/display_image_load.h>
#include <webots_ros/display_image_paste.h>

#define TIME_STEP 32
#define NMOTORS 2
#define MAX_SPEED 6.4
#define OBSTACLE_THRESHOLD 0.1
#define DECREASE_FACTOR 0.9
#define BACK_SLOWDOWN 0.9

ros::NodeHandle *n;

static int controllerCount;
static std::vector<std::string> controllerList;

ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;

static const char *motorNames[NMOTORS] = {"left_wheel_motor", "right_wheel_motor"};
static const char *motorNamesComplete[NMOTORS+1] = {"left_wheel_motor", "right_wheel_motor","servo"};
const std::string name = "change";
const std::string display = "display";
const std::string path_to_repo = "/Github/Robotics-project"

// gaussian function
double gaussian(double x, double mu, double sigma) {
  return (1.0 / (sigma * sqrt(2.0 * M_PI))) * exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma));
}

void maxSpeed() {
  // init variables
  double speeds[NMOTORS];

  // set speeds
  for (int i = 0; i < NMOTORS; ++i) {
    speeds[i] = MAX_SPEED;
    ros::ServiceClient set_velocity_client;
    webots_ros::set_float set_velocity_srv;
	  set_velocity_client = n->serviceClient<webots_ros::set_float>(name + std::string("/") + std::string(motorNames[i]) + std::string("/set_velocity"));
    set_velocity_srv.request.value = speeds[i];
    set_velocity_client.call(set_velocity_srv);
  }
}


void imageLoad(const std::string imageName) {
  ros::ServiceClient display_image_load_client;
  webots_ros::display_image_load display_image_load_srv;
  display_image_load_client = n.serviceClient<webots_ros::display_image_load>(name + "/display/image_load");

  display_image_load_srv.request.filename = std::string(getenv("HOME")) + path_to_repo +std::string("/Media/Image/")+ imageName +std::string(".jpg");
  display_image_load_client.call(display_image_load_srv);
  ROS_INFO("Image successfully loaded to clipboard.");
  uint64_t loaded_image = display_image_load_srv.response.ir;

  display_image_paste_srv.request.ir = loaded_image;
  if (display_image_paste_client.call(display_image_paste_srv) && display_image_paste_srv.response.success == 1)
    ROS_INFO("Image successfully load and paste.");
  else
    ROS_ERROR("Failed to call service display_image_paste to paste image.");

  display_image_load_client.shutdown();
  time_step_client.call(time_step_srv);
}

// play a .mp3 from filesystem with volume between 0.0 and 1.0 
void playSound(const std::string soundName, double volume, bool loop) {
  ros::ServiceClient speaker_play_sound_client;
  webots_ros::speaker_play_sound speaker_play_sound_srv;
  speaker_play_sound_client = n.serviceClient<webots_ros::speaker_play_sound>(name + "/speaker/play_sound");

  speaker_play_sound_srv.request.sound = std::string(getenv("HOME")) + path_to_repo +std::string("/Media/Audio/")+ soundName +std::string(".mp3");
  speaker_play_sound_srv.request.volume = volume;
  speaker_play_sound_srv.request.pitch = 1.0;
  speaker_play_sound_srv.request.balance = 0.0;
  speaker_play_sound_srv.request.loop = loop;
  
  if (speaker_play_sound_client.call(speaker_play_sound_srv) && speaker_play_sound_srv.response.success == 1)
    ROS_INFO("Sound successfully load and played.");
  else
    ROS_ERROR("Failed to call service speaker_play_sound to play a sound.");
  
  display_image_paste_client.shutdown();
  time_step_client.call(time_step_srv);
}


// speak a text with volume between 0.0 and 1.0 
void speak(const std::string &text, double volume) {
  ros::ServiceClient speaker_speak_client;
  webots_ros::speaker_speak speaker_speak_srv;
  speaker_speak_client = n.serviceClient<webots_ros::speaker_speak>(name + "/speaker/speak");

  speaker_speak_srv.request.text = text;
  speaker_speak_srv.request.volume = volume;
  
  if (speaker_speak_client.call(speaker_play_sound_srv) && speaker_speak_srv.response.success == 1)
    ROS_INFO("Text successfully readed.");
  else
    ROS_ERROR("Failed to call service speaker_speak to read a text.");
  
  speaker_speak_client.shutdown();
  time_step_client.call(time_step_srv);
}


// Set speaker's language
// 0 for American English.
// 1 for British English.
// 2 for German.
// 3 for Spanish.
// 4 for French.
// 5 for Italian.
void setLanguage(const int language) {
  const std::string languages[6] = {"en-US", "en-UK", "de-DE", "es-ES", "fr-FR", "it-IT"};
  ros::ServiceClient speaker_set_language_client;
  webots_ros::speaker_speak speaker_set_language_srv;
  speaker_set_language_client = n.serviceClient<webots_ros::set_string>(name + "/speaker/set_language");

  speaker_set_language_srv.request.value = languages[language];
  
  if (speaker_set_language_client.call(speaker_set_language_srv) && speaker_speak_srv.response.success == 1)
    ROS_INFO("Language successfully setted.");
  else
    ROS_ERROR("Failed to call service set_language to set speaker's language.");
  
  speaker_set_language_client.shutdown();
  time_step_client.call(time_step_srv);
}

// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr &name) {
  controllerCount++;
  controllerList.push_back(name->data);
  ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}

void quit(int sig) {
  ROS_INFO("User stopped the "+name+" node.");
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  std::string controllerName;
  // create a node named as "name" variable on ROS network
  ros::init(argc, argv, name, ros::init_options::AnonymousName);
  n = new ros::NodeHandle;

  signal(SIGINT, quit);

  // subscribe to the topic model_name to get the list of availables controllers
  ros::Subscriber nameSub = n->subscribe("model_name", 100, controllerNameCallback);
  while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
  }
  ros::spinOnce();

  timeStepClient = n->serviceClient<webots_ros::set_int>(name+"/robot/time_step");
  timeStepSrv.request.value = TIME_STEP;

  // if there is more than one controller available, it let the user choose
  if (controllerCount == 1)
    controllerName = controllerList[0];
  else {
    int wantedController = 0;
    std::cout << "Choose the # of the controller you want to use:\n";
    std::cin >> wantedController;
    if (1 <= wantedController && wantedController <= controllerCount)
      controllerName = controllerList[wantedController - 1];
    else {
      ROS_ERROR("Invalid number for controller choice.");
      return 1;
    }
  }
  ROS_INFO("Using controller: '%s'", controllerName.c_str());
  // leave topic once it is not necessary anymore
  nameSub.shutdown();

  // init motors
  for (int i = 0; i < NMOTORS+1; ++i) {
    // position
    ros::ServiceClient set_position_client;
    webots_ros::set_float set_position_srv;
    set_position_client = n->serviceClient<webots_ros::set_float>(name + std::string("/") + std::string(motorNamesComplete[i]) +
                                                                  std::string("/set_position"));

    set_position_srv.request.value = INFINITY;
    
    
    if (set_position_client.call(set_position_srv) && set_position_srv.response.success)
      ROS_INFO("Position set to INFINITY for motor %s.", motorNamesComplete[i]);
    else
      ROS_ERROR("Failed to call service set_position on motor %s.", motorNamesComplete[i]);

    // speed
    ros::ServiceClient set_velocity_client;
    webots_ros::set_float set_velocity_srv;
    set_velocity_client = n->serviceClient<webots_ros::set_float>(name + std::string("/") + std::string(motorNamesComplete[i]) +
                                                                  std::string("/set_velocity"));

    set_velocity_srv.request.value = 0.0;
    if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)
      ROS_INFO("Velocity set to 0.0 for motor %s.", motorNamesComplete[i]);
    else
      ROS_ERROR("Failed to call service set_velocity on motor %s.", motorNamesComplete[i]);
  }

  

  // enable accelerometer
  ros::ServiceClient set_accelerometer_client;
  webots_ros::set_int accelerometer_srv;
  ros::Subscriber sub_accelerometer;
  set_accelerometer_client = n->serviceClient<webots_ros::set_int>(name+"/accelerometer/enable");
  accelerometer_srv.request.value = 32;
  set_accelerometer_client.call(accelerometer_srv);

  // enable camera
  ros::ServiceClient set_camera_client;
  webots_ros::set_int camera_srv;
  ros::Subscriber sub_camera;
  set_camera_client = n->serviceClient<webots_ros::set_int>(name+"/camera/enable");
  camera_srv.request.value = 64;
  set_camera_client.call(camera_srv);

  // enable gyro
  ros::ServiceClient set_gyro_client;
  webots_ros::set_int gyro_srv;
  ros::Subscriber sub_gyro;
  set_gyro_client = n->serviceClient<webots_ros::set_int>(name+"/gyro/enable");
  gyro_srv.request.value = 32;
  set_gyro_client.call(gyro_srv);


  ROS_INFO("You can now visualize the sensors output in rqt using 'rqt'.");

  // for test
  maxSpeed();
  imageLoad("warning");
  playSound("warning");
  // main loop
  while (ros::ok()) {
    if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success) {
      ROS_ERROR("Failed to call service time_step for next step.");
      break;
    }
    ros::spinOnce();
  }
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);

  ros::shutdown();
  return 0;
}
