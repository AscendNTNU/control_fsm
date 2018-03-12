#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <ros/ros.h>
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

#define FIELD_LENGTH 20.0


struct Obstacle {
    float x;
    float y;
    float dir;
    Obstacle(float x, float y, float dir) : x(x), y(y), dir(dir){}
};

class PathPlanner;
class PathPlanner{
private:
    float obstacle_radius; //meters
    float node_distance; //meters
    float diagonal_node_distance; //meters
    int graph_size; //number of nodes in one direction

    std::vector<std::vector<Node>> graph;

    std::priority_queue<Node> open_list;
    //std::array<std::array<bool, graph_size>, graph_size> closed_list;

    std::list<Node> plan;
    std::list<Node> simple_plan;

    std::list<Obstacle> obstacles;

    Node end_node;
    Node start_node;

    bool no_plan = true;
    bool destination_reached = false;

    // Because of the NODE_DISTANCE, the destination found and the one requested
    // might not be exactly equal
    float destination_found_x;
    float destination_found_y;

    PathPlanner() = delete;
public:
    PathPlanner(float obstacle_radius, float node_distance);

    int coordToIndex(float coord);

    void initializeGraph();

    void setObstacles(std::list<Obstacle> &obstacles);

    // Add a circular obstacle
    void addObstacle(float center_x, float center_y);
    void refreshObstacles();
    Obstacle* findBlockingObstacle(float current_x, float current_y);

    // Adds unsafe zone around obstacles, the plan cannot be made here
    void addUnsafeZone(float center_x, float center_y);
    void refreshUnsafeZones();

    // Diagonal heuristic - can move in 8 directions from current point
    float calculateDiagonalHeuristic(float x, float y);
    float calculateManhattanHeuristic(float x, float y);
    float calculateEuclidianHeuristic(float x, float y);

    // Print the f-value of each node
    //void printGraph();
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
    bool canSimplifyLine(float x1, float y1, float x2, float y2);

    // Use A* to calculate the path
    void makePlan(float current_x, float current_y, float target_x, float target_y);
    // Same plan but with fewer points
    void simplifyPlan();
    // Verify the plan from where the drone is to the end point
    bool isPlanSafe(float current_x, float current_y);
    void resetParameters();
    bool removeOldPoints(float current_x, float current_y);

    // These functions are mainly for the visualization
    std::vector<std::vector<Node>> getGraph(){return graph;}
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
