#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "control/planner/path_planner.hpp"
#include "control/tools/drone_handler.hpp"
#include <list>
#include <vector>
#include "ascend_msgs/PathPlannerAction.h"
#include "ascend_msgs/GRStateArray.h"
#include <mavros_msgs/PositionTarget.h>

using control::pathplanner::PathPlanner;

// typedefs

using ActionServerType = actionlib::SimpleActionServer<ascend_msgs::PathPlannerAction>;

std::list<float> obstacle_coordinates;
float setpoint_x = 0;
float setpoint_y = 0;

struct PlannerState{
	float current_x, current_y, goal_x, goal_y;
	bool make_plan;
	bool new_goal;
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
	planner_state->new_goal = true;
	auto goal = server->acceptNewGoal();
	planner_state->goal_x = goal->x;
	planner_state->goal_y = goal->y;

	if(server->isPreemptRequested()) {
		//Goal is already stopped by client
		return;
	}
}


void updateObstaclesCB(ascend_msgs::GRStateArray::ConstPtr msg_p){
	obstacle_coordinates.clear();
	for(auto it = msg_p->states.begin(); it != msg_p->states.end(); ++it) {
    	obstacle_coordinates.push_back(it->x);
    	obstacle_coordinates.push_back(it->y);
	}
}

void updateSetpointCB(mavros_msgs::PositionTarget::ConstPtr msg_p){
	setpoint_x = msg_p->position.x;
	setpoint_y = msg_p->position.y;
}


int main(int argc, char** argv){

	PlannerState planner_state;
	PathPlanner plan;

    ros::init(argc, argv, "path_planner_server");
    ros::NodeHandle n;
    ros::Rate rate(30.0);
 
    ActionServerType server(n, "plan_path", false);

    server.registerGoalCallback(boost::bind(newPlanCB, &server, &planner_state));
    server.registerPreemptCallback(boost::bind(preemptCB, &server, &planner_state));
    server.start();

    ros::Subscriber sub_obstacles = n.subscribe("", 1, updateObstaclesCB);
    ros::Subscriber sub_setpoint = n.subscribe("", 1, updateSetpointCB);


    while(ros::ok()){

    	ros::spinOnce();

    	plan.refreshObstacles(obstacle_coordinates);

    	
    	if(planner_state.make_plan && (!plan.isPlanSafe(setpoint_x,setpoint_y) || planner_state.new_goal)){
    		ROS_INFO("Make new plan.");
    		planner_state.new_goal = false;
		    plan.makePlan(planner_state.current_x, planner_state.current_y, planner_state.goal_x, planner_state.goal_y);
		    std::list<Node> points = plan.getSimplePlan();
		    geometry_msgs::Point32 point;
		    int index = 0;
		    for(std::list<Node>::iterator it = points.begin(); it != points.end(); it++){
        		point.x = it->getX();
        		point.y = it->getY();

        		ascend_msgs::PathPlannerFeedback feedback;
        		feedback.points_in_plan[index++] = point;

        		server.publishFeedback(feedback);
    		}
    	}
    	else{
    		rate.sleep();
    	}
    }

    ros::spin();

}