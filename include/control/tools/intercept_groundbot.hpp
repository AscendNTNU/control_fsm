#ifndef INTERCEPT_GROUNDBOT_HPP
#define INTERCEPT_GROUNDBOT_HPP

typedef struct point{
	float x; 
	float y; 
	float z; 
} point;

point calculate_roomba_velocity(float roomba_x, float roomba_y, float roomba_z, ros::Time stamp);


mavros_msgs::PositionTarget InterceptGB(geometry_msgs::PoseStamped quad_position, geometry_msgs::PoseStamped roomba_position); 



#endif
