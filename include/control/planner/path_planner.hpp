#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <ros/ros.h>
#include <ascend_msgs/PathPlannerPlan.h>
#include "control/tools/control_pose.hpp"
#include <list>
#include <queue>
#include <math.h>
#include <iostream>
#include <iomanip>


#define FIELD_LENGTH 20


class Node{
private:
    float x;
    float y;

    // heuristic, use Euclidian distance
    float h;
    float g;
    // Sum of g and h
    float f;
public:
    Node();
    Node(float x, float y, float g, float h):x(x), y(y), g(g), h(h){f = g+h;}

    float getX() const {return x;}
    float getY() const {return y;}
    float getF() const {return f;}

    // Implemented only for the closed list priority queue
    friend bool operator< (const Node &lhs, const Node &rhs);
};


class PathPlanner{
private:
    Node* graph[FIELD_LENGTH][FIELD_LENGTH];

    std::priority_queue<Node> open_list;
    std::list<Node> closed_list;

    std::list<Node> plan;

    Node* end_node;
    Node* start_node;

public:
    PathPlanner(float current_x, float current_y, float target_x, float target_y);
    void initializeGraph();
    float calculateHeuristic(float x, float y);
    float calculateG(float x, float y);
    void printGraph();
};

#endif // PATH_PLANNER_HPP
