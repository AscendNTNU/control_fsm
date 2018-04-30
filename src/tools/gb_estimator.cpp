
#include <ros/ros.h>

#include "control/tools/gb_estimator.h"
#include <iostream>
using control::GbEstimator;

GbEstimator::GbEstimator(float gb_x, float gb_y){
	current_pos_x = gb_x; 
	current_pos_y = gb_y; 
	last_time_stamp = ros::Time::now(); 
	gb_speed = 0.33; 
	valid = true; 
}

GbEstimator::GbEstimator(){
	valid = false; 
}

void GbEstimator::update(const float *gb_direction, float *gb_position){
	if(!valid){
		ROS_ERROR_NAMED("GB Estimator", "Error, not valid groundbot!"); 
		return; 
	}
	ros::Duration delta_t;
	delta_t = ros::Time::now() - last_time_stamp;
	last_time_stamp = ros::Time::now(); 
		
	float velocity[2];

	velocity[0] = gb_direction[0]*gb_speed;
	velocity[1] = gb_direction[1]*gb_speed; 

	current_pos_x = current_pos_x + delta_t.toSec()*velocity[0]; 
	current_pos_y = current_pos_y + delta_t.toSec()*velocity[1]; 

	gb_position[0] = current_pos_x; 
	gb_position[1] = current_pos_y;		
}




