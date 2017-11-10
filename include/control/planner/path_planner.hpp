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
#include <array>
#include <memory>
#include <algorithm>
#include <limits>


#define FIELD_LENGTH 20


class Node{
private:
    float x;
    float y;

    float g;
    // heuristic, use diagonal distance
    float h;
    // Sum of g and h
    float f;

    float parent_x = -1;
    float parent_y = -1;
public:
    Node(){}
    Node(float x, float y, float g, float h):x(x), y(y), g(g), h(h){f = g+h;}

    float getX() const {return x;}
    float getY() const {return y;}
    float getF() const {return f;}
    float getG() const {return g;}
    float getParentX() const {return parent_x;}
    float getParentY() const {return parent_y;}

    void setG(float g) {this->g = g;}
    void setParentX(float parent_x){this->parent_x = parent_x;}
    void setParentY(float parent_y){this->parent_y = parent_y;}
    void updateF(float new_g);

    bool isValid();

    // Implemented only for the closed list priority queue
    friend bool operator< (const Node &lhs, const Node &rhs);
};


class PathPlanner{
private:
    std::array<std::array<Node, FIELD_LENGTH>, FIELD_LENGTH> graph;

    std::priority_queue<Node> open_list;
    std::array<std::array<bool, FIELD_LENGTH>, FIELD_LENGTH> closed_list;

    std::list<Node> plan;

    Node end_node;
    Node start_node;

    bool destination_reached = false;

public:
    PathPlanner(float current_x, float current_y, float target_x, float target_y);

    void initializeGraph();
    void initializeClosedList();

    float calculateEuclidianHeuristic(float x, float y);
    float calculateDiagonalHeuristic(float x, float y);

    void printGraph();
    void relaxGraph();

    void handleSuccessor(float x, float y, float parent_x, float parent_y,
                         float distance_to_parent);
    void handleAllSuccessors(float x, float y);
    bool isDestination(float x, float y);

    void makePlan();
};

#endif // PATH_PLANNER_HPP
