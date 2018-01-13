#include "control/planner/path_planner.hpp"


int coordToIndex(float k) {
    int index = static_cast<int>(k*10.0+0.5);
    return (index);
}



PathPlanner::PathPlanner(){}

void PathPlanner::initializeGraph(){
    for (float x = 0; x < FIELD_LENGTH; x += 0.1){
        for (float y = 0; y < FIELD_LENGTH; y += 0.1){
            if(isStart(x,y)){
                graph[coordToIndex(x)][coordToIndex(y)] = start_node;
                // Push the start node to the open list so that the search starts from the start
                open_list.push(graph[coordToIndex(x)][coordToIndex(y)]);
            }
            else {
                graph[coordToIndex(x)][coordToIndex(y)] = Node(x,y, std::numeric_limits<float>::infinity()
                    , calculateDiagonalHeuristic(x,y));
            }
        }
    }
}

void PathPlanner::initializeClosedList(){
    for (int i = 0; i < GRAPH_SIZE; i++) {
        for (int j = 0; j < GRAPH_SIZE; j++){
            closed_list[i][j] = false;
        }
    }
}

void PathPlanner::addObstacle(float center_x, float center_y) {
    // Find the y value for each x in the circle
    for (float x = center_x-OBSTACLE_RADIUS; x < center_x+OBSTACLE_RADIUS; x+=0.1) {
        float y = sqrt(OBSTACLE_RADIUS*OBSTACLE_RADIUS-(x-center_x)*(x-center_x));
        // Do this to fill the circle
        for(float i = center_y-y; i<=center_y+y; i+= 0.1) {
            if (isValidCoordinate(x, i)) {
                closed_list[coordToIndex(x)][coordToIndex(i)] = true;
            }
        }

    }
    // Do the same as over, just switch x and y
    for (float y = center_y-OBSTACLE_RADIUS; y < center_y+OBSTACLE_RADIUS; y+=0.1) {
        float x = sqrt(OBSTACLE_RADIUS*OBSTACLE_RADIUS-(y-center_y)*(y-center_y));
        for(float i = center_x-x; i<=center_x+x; i+= 0.1) {
            if (isValidCoordinate(i, y)) {
                closed_list[coordToIndex(i)][coordToIndex(y)] = true;
            }
        }
    }
}


float PathPlanner::calculateDiagonalHeuristic(float x, float y) {
    return (std::max(abs(x-end_node.getX()), abs(y-end_node.getY())));
}

void PathPlanner::printGraph(){
    for(int i = 0; i < GRAPH_SIZE; i++){
        for(int j = 0; j < GRAPH_SIZE; j++){
            std::cout << std::fixed;
            std::cout << std::setprecision(2);
            std::cout << graph[i][j].getF() << "  ";
        }
        std::cout << std::endl;
    }
}

void PathPlanner::relaxGraph(){
    Node current_node;
    while(!open_list.empty()){
        // Always search from the node with lowest f value
        current_node = open_list.top();
        open_list.pop();
        float x = current_node.getX();
        float y = current_node.getY();
        // Mark node as searched
        closed_list[coordToIndex(x)][coordToIndex(y)] = true;
        // Search all eight points around current point
        handleAllSuccessors(x,y);
        // Stop search if destination is reached
        if(destination_reached){return;}
    }
}

void PathPlanner::handleSuccessor(float x, float y, float parent_x, float parent_y, float distance_to_parent) {
    if(isValidCoordinate(x,y)){
        // Making sure the zero is actually zero and not something like 1.4895e-9
        if(x > 0.0 && x < 0.01){x = 0.0;}
        if(y > 0.0 && y < 0.01){y = 0.0;}
        if(isDestination(x,y)){
            graph[coordToIndex(x)][coordToIndex(y)].setParentX(parent_x);
            graph[coordToIndex(x)][coordToIndex(y)].setParentY(parent_y);
            destination_reached = true;
            std::cout << "Destination reached: x=" << x << " y=" << y << std::endl;
            std::cout << "Destination parent: x=" << graph[coordToIndex(x)][coordToIndex(y)].getParentX()
                 << " y=" << graph[coordToIndex(x)][coordToIndex(y)].getParentY() << std::endl;
        }
        // If node is not destination and hasn't been searched yet
        else if(closed_list[coordToIndex(x)][coordToIndex(y)] == false){
            // Calculate distance from start through the parent
            float new_g = graph[coordToIndex(parent_x)][coordToIndex(parent_y)].getG() + distance_to_parent;
            // Update graph and open list if distance is less than before through the parent
            if(graph[coordToIndex(x)][coordToIndex(y)].getG() > new_g) {
                graph[coordToIndex(x)][coordToIndex(y)].updateF(new_g);
                // For visualization
                if(max_f < graph[coordToIndex(x)][coordToIndex(y)].getF()){
                    max_f = graph[coordToIndex(x)][coordToIndex(y)].getF();
                }
                graph[coordToIndex(x)][coordToIndex(y)].setParentX(parent_x);
                graph[coordToIndex(x)][coordToIndex(y)].setParentY(parent_y);
                open_list.push(graph[coordToIndex(x)][coordToIndex(y)]);
            }
        }
    }
}

void PathPlanner::handleAllSuccessors(float x, float y) {
    // LOVER Å LØSE DETTE PÅ EN BEDRE MÅTE
    handleSuccessor(x+NODE_DISTANCE,y, x, y, NODE_DISTANCE);
    if(destination_reached){return;}
    handleSuccessor(x+NODE_DISTANCE, y+NODE_DISTANCE, x, y, DIAGONAL_NODE_DISTANCE);
    if(destination_reached){return;}
    handleSuccessor(x+NODE_DISTANCE, y-NODE_DISTANCE, x, y, DIAGONAL_NODE_DISTANCE);
    if(destination_reached){return;}
    handleSuccessor(x, y+NODE_DISTANCE, x, y, NODE_DISTANCE);
    if(destination_reached){return;}
    handleSuccessor(x, y-NODE_DISTANCE, x, y, NODE_DISTANCE);
    if(destination_reached){return;}
    handleSuccessor(x-NODE_DISTANCE, y, x, y, NODE_DISTANCE);
    if(destination_reached){return;}
    handleSuccessor(x-NODE_DISTANCE, y+NODE_DISTANCE, x, y, DIAGONAL_NODE_DISTANCE);
    if(destination_reached){return;}
    handleSuccessor(x-NODE_DISTANCE, y-NODE_DISTANCE, x, y, DIAGONAL_NODE_DISTANCE);
}

bool PathPlanner::isDestination(float x, float y) {
    return ((x < end_node.getX()+0.05 && x>end_node.getX()-0.05)
            && (y < end_node.getY()+0.05 && y > end_node.getY()-0.05));
}

bool PathPlanner::isStart(float x, float y) {
    return ((x < start_node.getX()+0.05 && x>start_node.getX()-0.05)
            && (y < start_node.getY()+0.05 && y > start_node.getY()-0.05));
}

bool PathPlanner::isValidCoordinate(float x, float y) {
    return(x>=0 && x<FIELD_LENGTH && y>=0 && y<FIELD_LENGTH);
}

void PathPlanner::makePlan(float current_x, float current_y, float target_x, float target_y) {
    
    start_node = Node(current_x, current_y, 0, 0);
    start_node.setParentX(0);
    start_node.setParentY(0);
    end_node = Node(target_x,target_y, std::numeric_limits<float>::infinity(), 0);
    initializeGraph();
    initializeClosedList();

    // Calculate all f values and set the parents
    relaxGraph();

    float x = end_node.getX();
    float y = end_node.getY();
    // Dummy safety for avoiding infinite loop, will remove later
    int counter = 0;
    // Go through the graph from end to start through the parents of each node
    // Add nodes to the plan
    while(!isStart(x, y)){
        plan.push_front(graph[coordToIndex(x)][coordToIndex(y)]);
        float x_temp = graph[coordToIndex(x)][coordToIndex(y)].getParentX();
        float y_temp = graph[coordToIndex(x)][coordToIndex(y)].getParentY();
        x = x_temp;
        y = y_temp;

        counter ++;
        if(counter >250){break;}
    }
    // Add start node to the plan (might not be necessary)
    plan.push_front(graph[coordToIndex(x)][coordToIndex(y)]);
    // Print the plan - can be removed
    /*for (std::list<Node>::iterator it = plan.begin(); it!= plan.end(); ++it){
        std::cout << "x: " << it->getX() << " y: " << it->getY() << std::endl;
    }*/

    // Delete unnecessary points
    simplifyPlan();
}

void PathPlanner::simplifyPlan() {
    /*std::list<Node>::iterator current = plan.begin();
    std::list<Node>::iterator next = plan.begin();
    next++;
    simple_plan.push_back(*current);
    while(next != plan.end()){
        if(coordToIndex(current->getX()) == coordToIndex(next->getX())){
            //std::cout << "VERTICAL" << std::endl;
            while(coordToIndex(current->getX()) == coordToIndex(next->getX()) && next != plan.end()){
                current++;
                next++;
            }
        }
        else if(coordToIndex(current->getY()) == coordToIndex(next->getY())){
            //std::cout << "HORISONTAL" << std::endl;
            while(coordToIndex(current->getY()) == coordToIndex(next->getY()) && next != plan.end()){
                current++;
                next++;
            }
        }
        else{
            //std::cout << "DIAGONAL" << std::endl;
            while(coordToIndex(current->getX()) != coordToIndex(next->getX()) && coordToIndex(current->getY()) != coordToIndex(next->getY()) && next != plan.end()){
                current++;
                next++;
            }
        }
        simple_plan.push_back(*current);
    }*/
    simple_plan = plan;
    // Pointing at the three first elements
    std::list<Node>::iterator first = simple_plan.begin();
    std::list<Node>::iterator second = simple_plan.begin();
    std::list<Node>::iterator third = simple_plan.begin();
    second++;
    third++;
    third++;

    float x1, x2, y1, y2, delta_x, delta_y, num_points;
    bool hit_obstacle = false;

    while(third != simple_plan.end()){
        // Parameters for finding the straight line between the first and third point
        x1 = first->getX();
        x2 = third->getX();
        y1 = first->getY();
        y2 = third->getY();
        delta_x = x2-x1;
        delta_y = y2-y1;
        num_points = 20*std::max(abs(delta_x),abs(delta_y));
        for(int i = 0; i < num_points; i++){
            x1 += delta_x/num_points;
            y1 += delta_y/num_points;
            // checking if the line goes through an obstacle at this point
            if(graph[coordToIndex(x1)][coordToIndex(y1)].obstacle){
                hit_obstacle = true;
            }
        }
        // If an obstacle is hit, the points are necessary and the search will start
        // at the end of the current search.
        if(hit_obstacle){
            second = first;
            second++;
            third = second;
            third++;
        }
        // If the straight line does not go through an obstacle, delete the second point
        // (no use in going through it when we can go in a straight line)
        // The search continues with same starting point, but the second and third point is changed
        else{
            simple_plan.erase(second);
            second = third;
            third++;
        }
        hit_obstacle = false;
    }

    // Print the remaining points
    for(std::list<Node>::iterator it = simple_plan.begin(); it != simple_plan.end(); it++){
        std::cout << "x: " << it->getX() << " y: " << it->getY() << std::endl;
    }
}



bool PathPlanner::isPlanSafe(float current_x, float current_y) {
    if(simple_plan.empty()){
        return false;
    }

    std::list<Node>::iterator current = simple_plan.begin();
    std::list<Node>::iterator next = current;
    next++;
    while(next != simple_plan.end()){
        if(((current_x < current->getX() && current_x > next->getX())
           || (current_x > current->getX() && current_x < next->getX()))
            && ((current_y < current->getY() && current_y > next->getY())
               || (current_y > current->getY() && current_y < next->getY()))){
            std::cout << "between " << current->getX() << " and " << next->getX() << std::endl;
            break;
        } else{
            simple_plan.erase(current);
            current = next;
            next++;
        }
    }

    current = simple_plan.begin();
    next = current;
    next++;
    while(next != simple_plan.end()){
        if(!isSafeLine(current->getX(), current->getY(), next->getX(), next->getY())){
            return false;
        }
        current++;
        next++;
    }
    return true;
}