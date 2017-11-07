#include "control/planner/path_planner.hpp"


bool operator< (const Node &lhs, const Node &rhs){
	return lhs.f > rhs.f;
}


PathPlanner::PathPlanner(float current_x, float current_y, float target_x, float target_y){
	start_node = std::unique_ptr<Node>(new Node (current_x, current_y, 0, 0));
    end_node =  std::unique_ptr<Node>(new Node (target_x, target_y, 0, 0));
    initializeGraph();
}

void PathPlanner::initializeGraph(){
    for (int x = 0; x < FIELD_LENGTH; x++){
        for (int y = 0; y < FIELD_LENGTH; y++){
            graph[x][y] = std::unique_ptr<Node> (new Node(x, y, calculateHeuristic(x,y), calculateG(x,y)));
            open_list.push(*(graph[x][y]));
        }
    }
}

float PathPlanner::calculateHeuristic(float x, float y){
    return (sqrt((x - end_node->getX())*(x - end_node->getX()) + (y - end_node->getY())*(y - end_node->getY())));
}

float PathPlanner::calculateG(float x, float y){
    return (sqrt((start_node->getX() - x)*(start_node->getX() - x) + (start_node->getY() - y)*(start_node->getY() - y)));
}

void PathPlanner::printGraph(){
    for(int x = 0; x < FIELD_LENGTH; x++){
        for(int y = 0; y < FIELD_LENGTH; y++){
            std::cout << std::fixed;
            std::cout << std::setprecision(2);
            std::cout << graph[x][y]->getF() << "  ";
        }
        std::cout << std::endl;
    }
}