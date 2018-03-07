#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "control/planner/path_planner.hpp"
#include "control/tools/drone_handler.hpp"
#include <list>
#include <vector>
#include "ascend_msgs/GRStateArray.h"
#include <mavros_msgs/PositionTarget.h>
#include "control/tools/planner_config.hpp"
#include "ascend_msgs/PointArrayStamped.h"
#include "ascend_msgs/PathPlanner.h"
#include "control/tools/obstacle_state_handler.hpp"

using control::pathplanner::PathPlanner;

struct PlannerState{
	float current_x, current_y, goal_x, goal_y;
	bool make_plan = false;
	bool new_goal = false;
};



using Request = ascend_msgs::PathPlanner::Request;
using Response = ascend_msgs::PathPlanner::Response;

bool newPlanCB(Request &req, Response &res, PlannerState* planner_state){

	if(req.cmd == Request::ABORT){
		ROS_INFO("Service callback: abort");
		planner_state->make_plan = false;
	}
	else if(req.cmd == Request::MAKE_PLAN){
		ROS_INFO("Service callback: make new plan");

		planner_state->make_plan = true;

		//Accept the new goal
		planner_state->new_goal = true;

		planner_state->goal_x = req.goal_x;
		planner_state->goal_y = req.goal_y;
	}
	else{
		ROS_INFO("Service callback: not relevant");
	}
	return true;
}


int main(int argc, char** argv){

	ros::init(argc, argv, "path_planner_server");
	ros::NodeHandle n;
    ros::Rate rate(5);

	ROS_INFO("Path planner node started");

	control::PlannerConfig::loadParams();

	PlannerState planner_state;
	PathPlanner plan(control::PlannerConfig::obstacle_radius, control::PlannerConfig::node_distance);

	std::list<float> obstacle_coordinates;

    ros::Publisher pub_plan = n.advertise<ascend_msgs::PointArrayStamped>(control::PlannerConfig::plan_points_topic, 1);
    ros::ServiceServer server = n.advertiseService<Request, Response>("path_planner_service", boost::bind(&newPlanCB, _1, _2, &planner_state));

    ascend_msgs::PointArrayStamped msg;
    auto& points_in_plan = msg.points;
    // For saving memory
	points_in_plan.reserve(20);

	while(!control::DroneHandler::isPoseValid() && control::ObstacleStateHandler::isInstanceReady() && ros::ok()){
		ros::spinOnce();
		ros::Duration(1.0).sleep();
	}

    while(ros::ok()){

    	ros::spinOnce();
    	if(planner_state.make_plan){
	    	auto obstacles = control::ObstacleStateHandler::getCurrentObstacles();
	    	obstacle_coordinates.clear();
			for(auto it = obstacles.global_robot_position.begin(); it != obstacles.global_robot_position.end(); ++it) {
		    	obstacle_coordinates.push_back(it->x);
		    	obstacle_coordinates.push_back(it->y);
			}
			plan.refreshObstacles(obstacle_coordinates);

			// Update planner state
			geometry_msgs::PoseStamped current_pose = control::DroneHandler::getCurrentPose();
			auto& position = current_pose.pose.position;
			planner_state.current_x = position.x;
			planner_state.current_y = position.y;
		}

    	plan.removeOldPoints(planner_state.current_x, planner_state.current_y);

    	// Make new plan as long as a plan is requested and the current one is invalid or the goal is changed
    	if(planner_state.make_plan && (!plan.isPlanSafe(planner_state.current_x,planner_state.current_y) || planner_state.new_goal)){
    		ROS_INFO("Make new plan.");
    		planner_state.new_goal = false;
		    plan.makePlan(planner_state.current_x, planner_state.current_y, planner_state.goal_x, planner_state.goal_y);
		    std::list<Node> simple_plan = plan.getSimplePlan();

		    geometry_msgs::Point point;
		    
		    // Removing old plan
		    points_in_plan.clear();
		    
		    if(!simple_plan.empty()){
			    
			    // The first point in the plan is the current point of the drone, so it doesn't need to be sent as part of the plan
			    std::list<Node>::iterator second_point = ++(simple_plan.begin());

			    std::cout << "Published points:\t";
			    for(std::list<Node>::iterator it = simple_plan.begin(); it != simple_plan.end(); it++){

	        		point.x = it->getX();
	        		point.y = it->getY();
	        		
	        		points_in_plan.push_back(point);

	        		std::cout << point.x << ", " << point.y << "\t";
	        	
	    		}
	    		std::cout << std::endl;
    		}
    	}
    
    	// Publish plan as long as a plan is requested
    	if(planner_state.make_plan){
    		std::cout << "Published points:\t";
    		for(auto it = points_in_plan.begin(); it != points_in_plan.end(); it++){
    			std::cout << it->x << ", " << it->y << "\t" << std::endl;
    		}
    		msg.header.stamp = ros::Time::now();
    		pub_plan.publish(msg);
    		std::cout << std::endl;
    	}

    	rate.sleep();
    	
    }

    ros::spin();

}