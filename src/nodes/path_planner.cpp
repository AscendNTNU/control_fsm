#include "control/planner/path_planner.hpp"


bool operator< (const Node &lhs, const Node &rhs){
	return lhs.f > rhs.f;
}


PathPlanner::PathPlanner(float32 current_x, float32 current_y, float32 target_x, float32 target_y){
	start_node = Node(current_x, current_y, 0, 0);
	end_node = Node(target_x, target_y, 0, 0);

}

void PathPlanner::initializeGraph(){
	for (int x = 0; x < FIELD_LENGTH; x++){
		for (int y = 0; y < FIELD_LENGTH; y++){
			graph[x][y] = Node(x, y, computeHeuristic(x,y), calculateG(x,y));
			open_list.push(graph[x][y]);
		}
	}
}

float32 PathPlanner::calculateHeuristic(float32 x, float32 y){
	return (sqrt((x - end_node.getX())*(x - end_node.getX()) + (y - end_node.getY())*(y - end_node.getY())));
}

float32 PathPlanner::calculateG(float32 x, float32 y){
	return(sqrt((start_node.getX() - x)*(start_node.getX() - x) + (start_node.getY() - y)*(start_node.getY() - y)));
}