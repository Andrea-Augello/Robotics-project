#ifndef __ROS_INTERFACE_H__
#define __ROS_INTERFACE_H__

#include <webots_ros/set_int.h>

#define TIME_STEP 32
#define N_MOTORS 2
#define N_LANGUAGES 6
#define MAX_SPEED 6.4
#define OBSTACLE_THRESHOLD 0.1
#define DECREASE_FACTOR 0.9
#define BACK_SLOWDOWN 0.9
#define N_DISTANCE_SENSORS 16

extern ros::NodeHandle *n;

extern ros::ServiceClient time_step_client;
extern webots_ros::set_int time_step_srv;

// gaussian function
double gaussian(double x, double mu, double sigma);

void enable_device(const std::string device);

void set_motor_position(const std::string motor, double position) ;

void set_motor_speed(const std::string motor, double speed) ;

// Get true if speaker is speaking, false otherwise
bool is_speaking() ;

// Show on the display an image from filesystem
void image_load(const std::string imageName) ;

// TO FIX
// play a .mp3 from filesystem with volume between 0.0 and 1.0 
void play_sound(const std::string soundName, double volume, bool loop) ;


// speak a text with volume between 0.0 and 1.0 
void speak(const std::string &text, double volume) ;


// Set speaker's language
// 0 for Italian.
// 1 for American English.
// 2 for German.
// 3 for Spanish.
// 4 for French.
// 5 for British English.

void set_language(const int language) ;

void speak_polyglot(const std::vector<std::string> text, double volume) ;

// catch names of the controllers availables on ROS network
void controller_name_callback(const std_msgs::String::ConstPtr &name) ;


void quit(int sig) ;

void init(int argc, char **argv);

#endif
