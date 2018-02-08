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

namespace control{
namespace pathplanner{

constexpr float OBSTACLE_RADIUS = 1.0; //meters
constexpr float NODE_DISTANCE = 0.4; // meters
constexpr float DIAGONAL_NODE_DISTANCE = sqrt(2*NODE_DISTANCE*NODE_DISTANCE); // meters

constexpr int GRAPH_SIZE = FIELD_LENGTH/NODE_DISTANCE+1.5; // number of nodes in one direction


class PathPlanner;
class PathPlanner{
private:
    std::array<std::array<Node, GRAPH_SIZE>, GRAPH_SIZE> graph;

    std::priority_queue<Node> open_list;
    std::array<std::array<bool, GRAPH_SIZE>, GRAPH_SIZE> closed_list;

    std::list<Node> plan;
    std::list<Node> simple_plan;

    std::list<float> obstacle_coordinates;

    Node end_node;
    Node start_node;

    bool noPlan = true;
    bool destination_reached = false;

public:
    PathPlanner();

    void initializeGraph();
    void initializeClosedList();

    // Add a circular obstacle
    void addObstacle(float center_x, float center_y);
    void refreshObstacles(std::list<float> obstacle_coordinates);

    // Diagonal heuristic - can move in 8 directions from current point
    float calculateDiagonalHeuristic(float x, float y);
    float calculateManhattanHeuristic(float x, float y);
    float calculateEuclidianHeuristic(float x, float y);

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
    bool isSafeLine(float x1, float y1, float x2, float y2);

    // Use A* to calculate the path
    void makePlan(float current_x, float current_y, float target_x, float target_y);
    // Same plan but with fewer points
    void simplifyPlan();
    // Verify the plan from where the drone is to the end point
    bool isPlanSafe(float current_x, float current_y);
    void resetParameters();

    // These functions are mainly for the visualization
    std::array<std::array<Node, GRAPH_SIZE>, GRAPH_SIZE> getGraph(){return graph;}
    std::list<Node> getPlan(){return plan;}
    std::list<Node> getSimplePlan(){return simple_plan;}
    // For the colors in the visualization
    float max_f = 0;
};

// Convert between coordinate x or y in metres and index in graph
//int coordToIndex(float k);

}
}

#endif // PATH_PLANNER_HPP
