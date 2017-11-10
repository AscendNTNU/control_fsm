#include "control/planner/path_planner.hpp"


bool operator< (const Node &lhs, const Node &rhs){
	return lhs.f > rhs.f;
}

bool Node::isValid(){
    return (x>0 && x<FIELD_LENGTH && y>0 && y<FIELD_LENGTH);
}

void Node::updateF(float new_g){
    if(new_g < g){
        g = new_g;
    }
    f = g+h;
}




PathPlanner::PathPlanner(float current_x, float current_y, float target_x, float target_y){
    start_node = Node(current_x, current_y, 0, 0);
    start_node.setParentX(0);
    start_node.setParentY(0);
    end_node = Node(target_x,target_y, std::numeric_limits<float>::infinity(), 0);
    initializeGraph();
    initializeClosedList();
}

void PathPlanner::initializeGraph(){
    for (int x = 0; x < FIELD_LENGTH; x++){
        for (int y = 0; y < FIELD_LENGTH; y++){
            if(x == start_node.getX() && y == start_node.getY()){
                graph[x][y] = start_node;
                open_list.push(graph[x][y]);
            }
            /*else if(x == end_node.getX() && y == end_node.getY()){
                graph[x][y] = end_node;
            }*/
            else {
                graph[x][y] = Node(x,y, std::numeric_limits<float>::infinity(), calculateDiagonalHeuristic(x,y));
            }
        }
    }
}

void PathPlanner::initializeClosedList(){
    for (int x = 0; x < FIELD_LENGTH; x++) {
        for (int y = 0; y < FIELD_LENGTH; y++){
            closed_list[x][y] = false;
        }
    }
}

float PathPlanner::calculateEuclidianHeuristic(float x, float y){
    return static_cast<float>(sqrt((x - end_node.getX()) * (x - end_node.getX()) +
                                   (y - end_node.getY()) * (y - end_node.getY())));
}

float PathPlanner::calculateDiagonalHeuristic(float x, float y) {
    return (std::max(abs(x-end_node.getX()), abs(y-end_node.getY())));
}

void PathPlanner::printGraph(){
    for(int x = 0; x < FIELD_LENGTH; x++){
        for(int y = 0; y < FIELD_LENGTH; y++){
            std::cout << std::fixed;
            std::cout << std::setprecision(2);
            std::cout << graph[x][y].getF() << "  ";
        }
        std::cout << std::endl;
    }
}

void PathPlanner::relaxGraph(){
    Node current_node;
    while(!open_list.empty()){
        current_node = open_list.top();
        open_list.pop();
        float x = current_node.getX();
        float y = current_node.getY();
        closed_list[x][y] = true;
        handleAllSuccessors(x,y);
        if(destination_reached) {return;}
    }
}

void PathPlanner::handleSuccessor(float x, float y, float parent_x, float parent_y, float distance_to_parent) {
    if(x>= 0 && x < FIELD_LENGTH && y >= 0 && y < FIELD_LENGTH){
        if(isDestination(x,y)){
            graph[x][y].setParentX(parent_x);
            graph[x][y].setParentY(parent_y);
            destination_reached = true;
            std::cout << "Destination reached: x=" << x << " y=" << y << std::endl;
        }
        else if(closed_list[x][y] == false){
            float new_g = graph[parent_x][parent_y].getG() + distance_to_parent;
            if(graph[x][y].getG() > new_g) {
                std::cout << "New node: " << x << ", " << y << " Parent: "
                          << parent_x << ", " << parent_y << std::endl;
                graph[x][y].updateF(new_g);
                graph[x][y].setParentX(parent_x);
                graph[x][y].setParentY(parent_y);
                open_list.push(graph[x][y]);
            }
        }
    }
}

void PathPlanner::handleAllSuccessors(float x, float y) {
    // LOVER Å LØSE DETTE PÅ EN BEDRE MÅTE!!!!!!!!!!!!!!!!!!!!
    handleSuccessor(x+1,y, x, y, 1);
    if(destination_reached){return;}
    handleSuccessor(x+1, y+1, x, y, sqrt(2));
    if(destination_reached){return;}
    handleSuccessor(x+1, y-1, x, y, sqrt(2));
    if(destination_reached){return;}
    handleSuccessor(x, y+1, x, y, 1);
    if(destination_reached){return;}
    handleSuccessor(x, y-1, x, y, 1);
    if(destination_reached){return;}
    handleSuccessor(x-1, y, x, y, 1);
    if(destination_reached){return;}
    handleSuccessor(x-1, y+1, x, y, sqrt(2));
    if(destination_reached){return;}
    handleSuccessor(x-1, y-1, x, y, sqrt(2));
}

bool PathPlanner::isDestination(float x, float y) {
    return ((x == end_node.getX()) && (y == end_node.getY()));
}

void PathPlanner::makePlan() {
    relaxGraph();

    float x = end_node.getX();
    float y = end_node.getY();
    
    while(!(x == start_node.getX() && y == start_node.getY())){
        plan.push_front(graph[x][y]);
        float x_temp = graph[x][y].getParentX();
        float y_temp = graph[x][y].getParentY();
        x = x_temp;
        y = y_temp;
    }
    plan.push_front(graph[x][y]);

    for (std::list<Node>::iterator it = plan.begin(); it!= plan.end(); ++it){
        std::cout << "x: " << it->getX() << " y: " << it->getY() << std::endl;
    }
}