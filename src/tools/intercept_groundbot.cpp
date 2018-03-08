#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>

#include "control/tools/config.hpp"
#include "control/tools/control_pose.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include "control/tools/drone_handler.hpp"
#include "control/tools/ground_robot_handler.hpp"
#include "control/tools/intercept_groundbot.hpp"


constexpr float PI_HALF = 1.570796; 
constexpr float roomba_speed = 0.35; 

uint16_t velocity_control = IGNORE_PX | IGNORE_PY | IGNORE_PZ |
	                    IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | 
	                    IGNORE_YAW_RATE;



/* Will return the a struct with the velocity (x,y,z) of the roomba in question. If the speed of the roomba
is greater than 0.35, the function will return the last velocity that was within
this bound. point is a struct with three variables, x,y and z.  
*/
point calculate_roomba_velocity(float roomba_x, float roomba_y, float roomba_z, ros::Time stamp){
	static auto last_time_stamp = stamp;
	static point last_roomba_pos = {roomba_x,roomba_y,roomba_z};
	static point velocity = {0.0, 0.0, 0.0}; 
	std::cout << last_roomba_pos.x << " " << last_roomba_pos.y << " " << last_roomba_pos.z << std::endl; 
	ros::Duration delta_t = stamp - last_time_stamp;  
	point temp;
	std::cout << "Time is " << delta_t.toSec() << std::endl; 
	if(delta_t.toSec() > 0.1){
		temp.x = (roomba_x - last_roomba_pos.x)/delta_t.toSec();
		temp.y = (roomba_y - last_roomba_pos.y)/delta_t.toSec();
		std::cout << "Temp x: " << temp.x << "Temp y" << temp.y << std::endl; 
		if(sqrt(pow(temp.x,2) + pow(temp.y, 2)) < roomba_speed){
			velocity.x = temp.x;
			velocity.y = temp.y; 
		}
		last_time_stamp = ros::Time::now(); 
	}
	last_roomba_pos.x = roomba_x; 
	last_roomba_pos.x = roomba_y; 
	last_roomba_pos.x = roomba_z; 

	return velocity; 
}

//Takes in quad message and the position of the roomba that is in question, return as velocity control message
mavros_msgs::PositionTarget InterceptGB(geometry_msgs::PoseStamped quad_position, ascend_msgs::GRState roomba_position){ 
	using control::Config;
	mavros_msgs::PositionTarget setpoint;	 
 
	auto quad_pose = quad_position.pose.position; 
	point roomba_pose = {roomba_position.x, roomba_position.y, 0.0};  

	point roomba_velocity = calculate_roomba_velocity(roomba_pose.x, roomba_pose.y, roomba_pose.z, roomba_position.header.stamp); 


	float distance_x = roomba_pose.x - quad_pose.x;  
	float distance_y = roomba_pose.y - quad_pose.y;
	float distance_z = roomba_pose.z - quad_pose.z; 


	float inner_product = pow(distance_x,2) + pow(distance_y, 2);
	float interception_gain = Config::tracking_param_xy/sqrt(pow(Config::interception_param_xy,2) + inner_product); 
	float interception_gain_z = Config::tracking_param_z/sqrt(pow(Config::interception_param_z,2) + inner_product);

	setpoint.yaw = control::pose::quat2mavrosyaw(quad_position.pose.orientation); 
	setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	setpoint.header.frame_id = "fcu"; 
	setpoint.type_mask = velocity_control;  

	setpoint.velocity.x = interception_gain * distance_x + roomba_velocity.x;
	setpoint.velocity.y = interception_gain * distance_y + roomba_velocity.y;
	setpoint.velocity.z = interception_gain_z * distance_z + roomba_velocity.z;

	// Some ultimate check to be sure!  



	return setpoint;
}




