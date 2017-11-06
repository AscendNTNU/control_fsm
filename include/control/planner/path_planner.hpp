#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <ros/ros.h>
#include <ascend_msgs/PathPlannerPlan.h>
#include "control/tools/control_pose.hpp"
#include <list>
#include <queue>
#include <math.h>


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
	Node(float32 x, float32 y, float32 g, float32 h):x(x), y(y), g(g), h(h){f = g+h;}
	// Implemented only for the closed list priority queue
	friend bool operator< (const Node &lhs, const Node &rhs);
	float32 getX() const {return x;}
	float32 getY() const {return y;}
};

class PathPlanner{
private:
	Node graph[FIELD_LENGTH][FIELD_LENGTH];

	std::priority_queue<Node> open_list;
	std::list<Node> closed_list;

	std::list<Node> plan;

	Node end_node;
	Node start_node;
	
	ascend_msgs::PathPlannerPlan current_plan_;

public:
    PathPlanner(float32 current_x, float32 current_y, float32 target_x, float32 target_y);
    void initializeGraph();
    float32 calculateHeuristic(float32 x, float32 y);
    float32 calculateG(float32 x, float32 y);
};

#endif // PATH_PLANNER_HPP
