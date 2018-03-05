#ifndef INTERCEPT_GROUNDBOT_HPP
#define INTERCEPT_GROUNDBOT_HPP

typedef struct point{
	float x; 
	float y; 
	float z; 
} point;


point roomba_velocity(float roomba_x, float roomba_y, float roomba_z);

mavros_msgs::PositionTarget InterceptGB(); 



#endif