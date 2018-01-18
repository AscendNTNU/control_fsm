#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "control/planner/path_planner.hpp"
#include "control/tools/drone_handler.hpp"
#include <list>
#include "ascend_msgs/PathPlannerAction.h"


// typedefs

using ActionServerType = actionlib::SimpleActionServer<ascend_msgs::PathPlannerAction>;



struct PlannerState{
	float current_x, current_y, goal_x, goal_y;
	bool make_plan;
};


//Terminates current goal when requested.
void preemptCB(ActionServerType* server, PlannerState* planner_state) {
	planner_state->make_plan = false;
	//Abort whatever you are doing first!
	ROS_WARN("Preempted!");
}

void newPlanCB(ActionServerType* server, PlannerState* planner_state){
	planner_state->make_plan = true;

	geometry_msgs::PoseStamped current_pose = control::DroneHandler::getCurrentPose();
	auto& position = current_pose.pose.position;
	planner_state->current_x = position.x;
	planner_state->current_y = position.y;

	if(!server->isNewGoalAvailable()){
		return;	
	}

	//Accept the new goal
	auto goal = server->acceptNewGoal();
	planner_state->goal_x = goal->x;
	planner_state->goal_y = goal->y;

	if(server->isPreemptRequested()) {
		//Goal is already stopped by client
		return;
	}
}


int main(int argc, char** argv){

	PlannerState planner_state;

    ros::init(argc, argv, "path_planner_server");
    ros::NodeHandle n;
    ros::Rate rate(30.0);
 
    ActionServerType server(n, "plan_path", false);

    server.registerGoalCallback(boost::bind(newPlanCB, &server, &planner_state));
    server.registerPreemptCallback(boost::bind(preemptCB, &server, &planner_state));
    server.start();

    ROS_INFO("Hello world!");

    while(ros::ok()){
    	ros::spinOnce();
    	if(planner_state.make_plan){
		    PathPlanner plan;
		    plan.makePlan(planner_state.current_x, planner_state.current_y, planner_state.goal_x, planner_state.goal_y);
		    std::list<Node> points = plan.getSimplePlan();
		    geometry_msgs::Point32 point;
		    int index = 0;
		    for(std::list<Node>::iterator it = points.begin(); it != points.end(); it++){
        		point.x = it->getX();
        		point.y = it->getY();

        		ascend_msgs::PathPlannerFeedback feedback;
        		feedback.points_in_plan[index] = point;

        		server.publishFeedback(feedback);
    		}
    	}
    	else{
    		rate.sleep();
    	}
    }

    ros::spin();

}