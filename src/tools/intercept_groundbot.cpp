#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>

#include "control/tools/config.hpp"
#include "control/tools/control_pose.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include "control/tools/drone_handler.hpp"
#include "control/tools/ground_robot_handler.hpp"
#include "control/tools/intercept_groundbot.hpp"

using control::Config;
using geometry_msgs::PoseStamped;
using ascend_msgs::GRState;
using mavros_msgs::PositionTarget;
using namespace control::gb;


constexpr float PI_HALF = 1.570796; 
constexpr float roomba_speed = 0.35; 

uint16_t velocity_control = IGNORE_PX | IGNORE_PY | IGNORE_PZ |
	                    IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | 
	                    IGNORE_YAW_RATE;

struct point{
	float x,y,z; 
};

point calculateRoombaVelocity(float roomba_x, float roomba_y, float roomba_z, ros::Time stamp, int& wrong_measurements);
//mavros_msgs::PositionTarget failsafe(mavros_msgs::PositionTarget setpoint);

/* Will return the a struct with the velocity (x,y,z) of the roomba in question. If the speed of the roomba
is greater than 0.35, the function will return the last velocity that was within
this bound. point is a struct with three variables, x,y and z.  
*/
point calculateRoombaVelocity(float roomba_x, float roomba_y, float roomba_z, ros::Time stamp, int& wrong_measurements){
	static auto last_time_stamp = stamp;
	static point last_roomba_pos = {roomba_x,roomba_y,roomba_z};
	static point velocity = {0.0, 0.0, 0.0}; 
	ros::Duration delta_t = stamp - last_time_stamp;  
	point temp;
	if(delta_t.toSec() > 0.1){
		temp.x = (roomba_x - last_roomba_pos.x)/delta_t.toSec();
		temp.y = (roomba_y - last_roomba_pos.y)/delta_t.toSec();
		if(sqrt(pow(temp.x,2) + pow(temp.y, 2)) < roomba_speed){
			velocity.x = temp.x;
			velocity.y = temp.y; 
		}else{
			wrong_measurements++; 
		}
		last_time_stamp = ros::Time::now(); 
	}
	last_roomba_pos.x = roomba_x; 
	last_roomba_pos.x = roomba_y; 
	last_roomba_pos.x = roomba_z; 

	return velocity; 
}


//Takes in quad message and the position of the roomba that is in question, return as velocity control message
bool control::gb::interceptGB(const PoseStamped& quad_position, const GRState& roomba_position, PositionTarget& setpoint){ 
	mavros_msgs::PositionTarget setpoint_temp;	 
 	static int wrong_measurements = 0; 

	auto quad_pose = quad_position.pose.position; 
	point roomba_pose = {roomba_position.x, roomba_position.y, 0.0};  

	point roomba_velocity = calculateRoombaVelocity(roomba_pose.x, roomba_pose.y, roomba_pose.z, roomba_position.header.stamp, wrong_measurements); 

	float distance_x = roomba_pose.x - quad_pose.x;  
	float distance_y = roomba_pose.y - quad_pose.y;
	float distance_z = roomba_pose.z - quad_pose.z; 

	if(distance_x > Config::max_distance || distance_y > Config::max_distance || distance_z > Config::max_distance){
		wrong_measurements++;
		if(wrong_measurements > 10){
			wrong_measurements = 0; 
		return false; 
		}  
	}


	float inner_product = pow(distance_x,2) + pow(distance_y, 2);
	float interception_gain = Config::tracking_param_xy/sqrt(pow(Config::interception_param_xy,2) + inner_product); 
	float interception_gain_z = Config::tracking_param_z/sqrt(pow(Config::interception_param_z,2) + inner_product);

	setpoint_temp.yaw = control::pose::quat2mavrosyaw(quad_position.pose.orientation); 
	setpoint_temp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	setpoint_temp.header.frame_id = "fcu"; 
	setpoint_temp.type_mask = velocity_control;  

	setpoint_temp.velocity.x = interception_gain * distance_x + roomba_velocity.x;
	setpoint_temp.velocity.y = interception_gain * distance_y + roomba_velocity.y;
	setpoint_temp.velocity.z = interception_gain_z * distance_z + roomba_velocity.z;


	wrong_measurements = 0; 
	setpoint = setpoint_temp; 
	return true;
}





