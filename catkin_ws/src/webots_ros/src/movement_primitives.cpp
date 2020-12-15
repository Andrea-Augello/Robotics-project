#include "ros_interface.hpp"
#include "movement_primitives.hpp"
#include <opencv2/opencv.hpp>

double angular_velocity	 = 3.0;
double linear_velocity	 = 3.0;

void rotate(double rotation, double precision){
	/*
	 * rotation:	The desired rotation in degrees. Note that a rotation 
	 * 					greater than 180° in modulo will be substituted with
	 * 					a rotation in the opposite direction.
	 * precision:	The	maximum difference from the required rotation and the
	 * 					actual rotation. If this value is smaller than the
	 * 					sensor noise it makes no sense.
	 */
	stop();
	double curr_angular_velocity = angular_velocity;
	int direction, framenum=1;
	processCallbacks();
	//update_frame();
	//update_object_roi();
	double difference,
		   current_angle = get_angle(),
		   target_angle=rotation+get_angle();
	// adjust for discontinuity at +/-180°
	target_angle = target_angle >  180 ? target_angle-360 : target_angle;
	target_angle = target_angle < -180 ? 360+target_angle : target_angle;
	difference = target_angle-current_angle;
	if(difference > 180){
		difference = difference - 360;
	} else if (difference < -180){
		difference = difference + 360;
	}
	direction = (difference > 0 ? 1 : -1);
	while(abs(difference)>precision){
		//update_frame();
		framenum = (framenum+1)%100;
		if(!framenum){
			//std::cout << "error:          " << difference << std::endl;
			if(cv::waitKey(1) == ' '){
				break;
			}
		}
		current_angle = get_angle();
		difference = target_angle-current_angle;
		if(difference > 180){
			difference = difference - 360;
		} else if (difference < -180){
			difference = difference + 360;
		}
		if(direction*difference < 0){
			// if we went over the specified angle, reverses the direction and
			// decreases the angular speed to have a better chance of sampling
			// at the right moment.
			curr_angular_velocity*=0.8;
			direction=-direction;
		}
		set_angular_velocity(curr_angular_velocity*(-direction));
	}
	//std::cout << "target angle:  " << target_angle  <<std::endl;
	stop();
}
