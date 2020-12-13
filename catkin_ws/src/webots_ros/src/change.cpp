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

#define TIME_STEP 32
#define N_MOTORS 2
#define N_LANGUAGES 6
#define MAX_SPEED 6.4
#define OBSTACLE_THRESHOLD 0.1
#define DECREASE_FACTOR 0.9
#define BACK_SLOWDOWN 0.9
#define N_DISTANCE_SENSORS 16

ros::NodeHandle *n;

static int controller_count;
static std::vector<std::string> controller_list;

ros::ServiceClient time_step_client;
webots_ros::set_int time_step_srv;

static const char *distance_sensor_names[N_DISTANCE_SENSORS] = {"ds0", "ds1", "ds2", "ds3", "ds4", "ds5", "ds6", "ds7", "ds8", "ds9", "ds10", "ds11", "ds12", "ds13", "ds14", "ds15"};
static const char *motor_names[N_MOTORS] = {"left_wheel_motor", "right_wheel_motor"};
static const char *motor_names_complete[N_MOTORS+1] = {"left_wheel_motor", "right_wheel_motor","servo"};
const std::string name = "change";


// gaussian function
double gaussian(double x, double mu, double sigma) {
  return (1.0 / (sigma * sqrt(2.0 * M_PI))) * exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma));
}

void enable_sensors(){
  ros::ServiceClient enable_sensor_client;
  webots_ros::set_int enable_sensor_srv;
  for (int i = 0; i < N_DISTANCE_SENSORS; ++i) {
    enable_sensor_client = n->serviceClient<webots_ros::set_int>(name + std::string("/") + std::string(distance_sensor_names[i]) + std::string("/enable"));
    if(enable_sensor_client.call(enable_sensor_srv)){
      ROS_INFO("Distance sensor %s succefully enabled.", distance_sensor_names[i]);
    }else{
      ROS_ERROR("Failed to call service enable_sensor.");
    }

  }
  enable_sensor_client.shutdown();
  time_step_client.call(time_step_srv); 
}

void set_motor_speed(double speed_left, double speed_right) {
  // init variables
  double speeds[N_MOTORS]= {speed_left, speed_right};

  // set speeds
  for (int i = 0; i < N_MOTORS; ++i) {
    ros::ServiceClient set_velocity_client;
    webots_ros::set_float set_velocity_srv;
	  set_velocity_client = n->serviceClient<webots_ros::set_float>(name + std::string("/") + std::string(motor_names[i]) + std::string("/set_velocity"));
    set_velocity_srv.request.value = speeds[i];
    set_velocity_client.call(set_velocity_srv);
  }
}

void spin_servo(double speed) {
  // set speeds
  ros::ServiceClient set_velocity_client;
  webots_ros::set_float set_velocity_srv;
  set_velocity_client = n->serviceClient<webots_ros::set_float>(name + std::string("/servo/set_velocity"));
  set_velocity_srv.request.value = speed;
  set_velocity_client.call(set_velocity_srv);

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

int main(int argc, char **argv) {
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
      return 1;
    }
  }
  ROS_INFO("Using controller: '%s'", controllerName.c_str());
  // leave topic once it is not necessary anymore
  nameSub.shutdown();

  // init motors
  for (int i = 0; i < N_MOTORS+1; ++i) {
    // position
    ros::ServiceClient set_position_client;
    webots_ros::set_float set_position_srv;
    set_position_client = n->serviceClient<webots_ros::set_float>(name + std::string("/") + std::string(motor_names_complete[i]) +
                                                                  std::string("/set_position"));

    set_position_srv.request.value = INFINITY;
    
    
    if (set_position_client.call(set_position_srv) && set_position_srv.response.success)
      ROS_INFO("Position set to INFINITY for motor %s.", motor_names_complete[i]);
    else
      ROS_ERROR("Failed to call service set_position on motor %s.", motor_names_complete[i]);

    // speed
    ros::ServiceClient set_velocity_client;
    webots_ros::set_float set_velocity_srv;
    set_velocity_client = n->serviceClient<webots_ros::set_float>(name + std::string("/") + std::string(motor_names_complete[i]) +
                                                                  std::string("/set_velocity"));

    set_velocity_srv.request.value = 0.0;
    if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)
      ROS_INFO("Velocity set to 0.0 for motor %s.", motor_names_complete[i]);
    else
      ROS_ERROR("Failed to call service set_velocity on motor %s.", motor_names_complete[i]);
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
  set_motor_speed(MAX_SPEED/4,MAX_SPEED/4);
  spin_servo(MAX_SPEED/4);
  enable_sensors();
  image_load("warning");
  set_language(0);
  speak("Ciao sono ciangà e sugnu troppu fuoitti",1.0);
  speak("Mi chiamo ROBOT SENZA INGEGNO",1.0);
  std::vector<std::string> ciao {"Ciao", "Hello", "Halo", "Hola", "Salut"};
  speak_polyglot(ciao, 1.0);
  
  /* TO FIX */
  //play_sound("warning", 1.0, 0);
  
  
  
  // main loop
  while (ros::ok()) {
    if (!time_step_client.call(time_step_srv) || !time_step_srv.response.success) {
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
