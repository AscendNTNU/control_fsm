#include <ros/ros.h>
#include "ascend_msgs/PathPlanner.h"


int main(int argc, char** argv){

	ROS_INFO("Test path planner node started");

	using Request = ascend_msgs::PathPlanner::Request;
	
	ros::init(argc, argv, "path_planner_test");
    ros::NodeHandle n;
    ros::Rate rate(30.0);

    ros::ServiceClient client  = n.serviceClient<ascend_msgs::PathPlanner>("path_planner_service");

    ascend_msgs::PathPlanner clientCall;

    clientCall.request.cmd = Request::MAKE_PLAN;
    clientCall.request.goal_x = 1;
	clientCall.request.goal_y = 1;

	if(client.call(clientCall)){
		ROS_INFO("Request handled");
	}
	else{
		ROS_WARN("Error!");
	}
 	
 	/*while(ros::ok()){
 		ROS_INFO("Ros is ok");
 		ros::spin();
 	}

	ros::Duration(3).sleep();

	clientCall.request.goal_x = 1;
	clientCall.request.goal_y = 8;

	if(client.call(clientCall)){
		ROS_INFO("Request handled");
	}
	else{
		ROS_WARN("Error!");
	}*/

}
