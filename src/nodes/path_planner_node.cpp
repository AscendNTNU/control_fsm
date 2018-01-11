#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "control/planner/path_planner.hpp"

// typedefs

using ActionServerType = actionlib::SimpleActionServer<ascend_msgs::PathPlannerAction>;


//Terminates current goal when requested.
void preemptCB(ActionServerType* server) {
	//Abort whatever you are doing first!
	ROS_WARN("Preempted!");
}

void newPlanCB(ActionServerType* server){
	

	if(server->isPreemptRequested()) {
		//Goal is already stopped by client
		return;
	}
}


int main(int argc, char** argv){

	int make_plan 

    ros::init(argc, argv, "path_planner_server");
    ros::NodeHandle n;
 
    ActionServerType server(n, "plan_path", false);

    server.registerPlanCallback(boost::bind(newPlanCB, &server));
    server.registerPreemptCallback(boost::bind(preemptCB, &server));
    server.start();

    ROS_INFO("Hello world!");

    ros::spin();

}