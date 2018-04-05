#include <ros/ros.h>
namespace control {

class GbEstimator{
private:
	float current_pos_x; 
	float current_pos_y; 
	float gb_speed; 
	ros::Time last_time_stamp; 
	bool valid; 

public: 
	GbEstimator(float gb_x, float gb_y);
	GbEstimator(); 
	void update(const float *gb_direction, float *gb_position); 
};
}