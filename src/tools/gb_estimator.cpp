
#include <ros/ros.h>

#include "gb_estimator.h"
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
		std::cout << "Error, not valid groundbot!" << std::endl; 
		return; 
	}
	ros::Duration time_step; 
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




