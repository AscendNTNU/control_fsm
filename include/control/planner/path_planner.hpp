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

#define GRAPH_SIZE 200 // number of nodes in one direction
#define OBSTACLE_RADIUS 0.8 // meters

class PathPlanner{
private:
    std::array<std::array<Node, GRAPH_SIZE>, GRAPH_SIZE> graph;

    std::priority_queue<Node> open_list;
    std::array<std::array<bool, GRAPH_SIZE>, GRAPH_SIZE> closed_list;

    std::list<Node> plan;
    std::list<Node> simple_plan;

    Node end_node;
    Node start_node;

    bool destination_reached = false;

public:
    PathPlanner(float current_x, float current_y, float target_x, float target_y);

    void initializeGraph();
    void initializeClosedList();

    // Add a circular obstacle
    void addObstacle(float center_x, float center_y);

    // Diagonal heuristic - can move in 8 directions from current point
    float calculateDiagonalHeuristic(float x, float y);

    // Print the f-value of each node
    void printGraph();
    void relaxGraph();

    //
    void handleSuccessor(float x, float y, float parent_x, float parent_y,
                         float distance_to_parent);
    // Every point has 8 succsessors
    void handleAllSuccessors(float x, float y);

    bool isDestination(float x, float y);
    bool isStart(float x, float y);
    bool isValidCoordinate(float x, float y);

    // Use A* to calculate the path
    void makePlan();
    // Same plan but with fewer points
    void simplifyPlan();

    // These functions are mainly for the visualization
    std::array<std::array<Node, FIELD_LENGTH*10>, FIELD_LENGTH*10> getGraph(){return graph;}
    std::list<Node> getPlan(){return plan;}
    // For the colors in the visualization
    float max_f = 0;
};

// Convert between coordinate x or y in metres and index in graph
int coordToIndex(float k);


#endif // PATH_PLANNER_HPP
