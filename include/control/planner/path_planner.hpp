#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <ros/ros.h>
#include <ascend_msgs/PathPlannerPlan.h>
#include "control/planner/node.hpp"
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
