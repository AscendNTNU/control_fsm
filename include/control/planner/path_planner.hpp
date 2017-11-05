#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <ros/ros.h>
#include <ascend_msgs/PathPlannerPlan.h>
#include "control/tools/control_pose.hpp"
#include <list>


#define FIELD_LENGTH 20



class Node{
private:
	float32 x;
	float32 y;

	// heuristic, use Euclidian distance
	float32 h;
	float32 g;
	// Sum of g and h
	float32 f;
public:
	Node(float32 x, float32 y):x(x), y(y){}4
};

class PathPlanner{
private:
	Node graph[FIELD_LENGTH][FIELD_LENGTH];

	std::list<Node> open_list;
	std::list<Node> closed_list;

	std::list<Node> plan;

	Node end_node;
	Node start_node;
	
	ascend_msgs::PathPlannerPlan current_plan_;

public:
    PathPlanner(float32 current_x, float32 current_y, float32 target_x, float32 target_y);
};

#endif // PATH_PLANNER_HPP
