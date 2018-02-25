#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "control/planner/path_planner.hpp"
#include "control/tools/drone_handler.hpp"
#include <list>
#include <vector>
#include "ascend_msgs/PathPlannerAction.h"
#include "ascend_msgs/GRStateArray.h"
#include <mavros_msgs/PositionTarget.h>
#include "control/tools/planner_config.hpp"
#include "ascend_msgs/PointArrayStamped.h"
#include "ascend_msgs/PathPlanner.h"

using control::pathplanner::PathPlanner;

// typedefs

using ActionServerType = actionlib::SimpleActionServer<ascend_msgs::PathPlannerAction>;

std::list<float> obstacle_coordinates;

struct PlannerState{
	float current_x, current_y, goal_x, goal_y;
	bool make_plan;
	bool new_goal;
};



using Request = ascend_msgs::PathPlanner::Request;
using Response = ascend_msgs::PathPlanner::Response;

bool newPlanCB(Request &req, Response &res, PlannerState* planner_state){

	ROS_INFO("newPlanCB");

	planner_state->make_plan = true;

	// Update planner state
	geometry_msgs::PoseStamped current_pose = control::DroneHandler::getCurrentPose();
	auto& position = current_pose.pose.position;
	planner_state->current_x = position.x;
	planner_state->current_y = position.y;

	//Accept the new goal
	planner_state->new_goal = true;

	planner_state->goal_x = req.goal_x;
	planner_state->goal_y = req.goal_y;

	return true;
}


void updateObstaclesCB(ascend_msgs::GRStateArray::ConstPtr msg_p){
	obstacle_coordinates.clear();
	for(auto it = msg_p->states.begin(); it != msg_p->states.end(); ++it) {
    	obstacle_coordinates.push_back(it->x);
    	obstacle_coordinates.push_back(it->y);
	}
}


int main(int argc, char** argv){

	ROS_INFO("Path planner node started");

	PlannerState planner_state;
	PathPlanner plan;

    ros::init(argc, argv, "path_planner_server");
    ros::NodeHandle n;
    ros::Rate rate(1);
 
    control::PlannerConfig::loadParams();


    //ros::Subscriber sub_obstacles = n.subscribe(control::PlannerConfig::obstacle_state_topic, 1, updateObstaclesCB);

    ros::Publisher pub_plan = n.advertise<ascend_msgs::PointArray>(control::PlannerConfig::plan_points_topic, 1);

    ros::ServiceServer server = n.advertiseService<Request, Response>("path_planner_service", boost::bind(&newPlanCB, _1, _2, &planner_state));

    ascend_msgs::PointArrayStamped msg;
    geometry_msgs::Point points_in_plan;

    while(ros::ok()){

    	ros::spinOnce();
    	

    	//plan.refreshObstacles(obstacle_coordinates);
    	plan.removeOldPoints(planner_state.current_x, planner_state.current_y);

    	// Make new plan as long as a plan is requested and the current one is invalid or the goal is changed
    	if(planner_state.make_plan && (!plan.isPlanSafe(planner_state.current_x,planner_state.current_y) || planner_state.new_goal)){
    		//ROS_INFO("Make new plan.");
    		planner_state.new_goal = false;
		    plan.makePlan(planner_state.current_x, planner_state.current_y, planner_state.goal_x, planner_state.goal_y);
		    std::list<Node> simple_plan = plan.getSimplePlan();

		    geometry_msgs::Point point;
		    // Iterate through the points in the plan and publish to pathplanner-action
		    ascend_msgs::PathPlannerFeedback feedback;
		    // For saving memory
		    feedback.points_in_plan.reserve(20);
		    points_in_plan.points.reserve(20);
		    
		    // The first point in the plan is the current point of the drone, so it doesn't need to be sent as part of the plan
		    std::list<Node>::iterator second_point = ++(simple_plan.begin());

		    std::cout << "Published points:\t";
		    for(std::list<Node>::iterator it = second_point; it != simple_plan.end(); it++){

        		point.x = it->getX();
        		point.y = it->getY();
        		
        		points_in_plan.points.push_back(point);

        		std::cout << point.x << ", " << point.y << "\t";
        	
    		}
    		msg.points = points_in_plan;
    		msg.header.stamp = ros::Time::now();
    		pub_plan.publish(msg);

    		std::cout << std::endl;
    	}
    
    	// Publish plan as long as a plan is requested
    	if(planner_state.make_plan){
    		std::cout << "Published points:\t";
    		for(auto it = points_in_plan.points.begin(); it != points_in_plan.points.end(); it++){
    			std::cout << it->x << ", " << it->y << "\t" << std::endl;
    		}
    		pub_plan.publish(points_in_plan);
    		std::cout << std::endl;
    	}

    	rate.sleep();
    	
    }

    ros::spin();

}