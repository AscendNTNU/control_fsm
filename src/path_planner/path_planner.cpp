#include "control/planner/path_planner.hpp"
#include "control/tools/obstacle_math.hpp"


using namespace control::pathplanner;


PathPlanner::PathPlanner(float obstacle_radius, float node_distance)
    : obstacle_radius(obstacle_radius), node_distance(node_distance){
    diagonal_node_distance = sqrt(2*node_distance*node_distance);
    // plus 1 to include both 0 and the last node
    graph_size = ceil(FIELD_LENGTH/node_distance+1);

    graph.resize(graph_size, std::vector<Node>(graph_size));
}

int PathPlanner::coordToIndex(float coord) {
    int index = round(coord/node_distance);
    return (index);
}

void PathPlanner::initializeGraph(){
    std::cout << "Init" << std::endl;
    for (float x = 0; x < FIELD_LENGTH; x += node_distance){
        for (float y = 0; y < FIELD_LENGTH; y += node_distance){
            if(isStart(x,y)){
                graph[coordToIndex(x)][coordToIndex(y)] = start_node;
                // Push the start node to the open list so that the search starts from the start
                open_list.push(graph[coordToIndex(x)][coordToIndex(y)]);
            }
            else {
                graph[coordToIndex(x)][coordToIndex(y)] =
                        Node(x,y, std::numeric_limits<float>::infinity(), calculateEuclidianHeuristic(x,y));
            }
        }
    }
}

void PathPlanner::setObstacles(std::list<Obstacle> &obstacles) {
    this->obstacles = obstacles;
}

void PathPlanner::addObstacle(float center_x, float center_y) {
    // Find the y value for each x in the circle
    for (float x = center_x-obstacle_radius; x <= center_x+obstacle_radius; x+=node_distance) {
        float y = sqrt(obstacle_radius*obstacle_radius-(x-center_x)*(x-center_x));
        // Do this to fill the circle
        for(float i = center_y-y; i<=center_y+y; i+= node_distance) {
            if (isValidCoordinate(x, i)) {
                graph[coordToIndex(x)][coordToIndex(i)].closed = true;
                graph[coordToIndex(x)][coordToIndex(i)].obstacle = true;
            }
        }

    }
    // Do the same as over, just switch x and y
    for (float y = center_y-obstacle_radius; y <= center_y+obstacle_radius; y+=node_distance) {
        float x = sqrt(obstacle_radius*obstacle_radius-(y-center_y)*(y-center_y));
        for(float i = center_x-x; i<=center_x+x; i+= node_distance) {
            if (isValidCoordinate(i, y)) {
                graph[coordToIndex(i)][coordToIndex(y)].closed = true;
                graph[coordToIndex(i)][coordToIndex(y)].obstacle = true;
            }
        }
    }
}

void PathPlanner::refreshObstacles() {
    std::cout << "refreshObstacles" << std::endl;
    if(no_plan){
        return;
    }

    // delete all old obstacles
    for (float x = 0; x < FIELD_LENGTH; x += node_distance){
        for (float y = 0; y < FIELD_LENGTH; y += node_distance){
                graph[coordToIndex(x)][coordToIndex(y)].obstacle = false;
        }
    }

    for(auto current = obstacles.begin(); current != obstacles.end(); current++){
        addObstacle(current->x, current->y);
    }
}

void PathPlanner::addUnsafeZone(float center_x, float center_y) {
    // Same logic as adding obstacles, only difference is the radius
    float unsafe_radius = obstacle_radius*2;
    // Find the y value for each x in the circle
    for (float x = center_x-unsafe_radius; x <= center_x+unsafe_radius; x+=node_distance) {
        float y = sqrt(unsafe_radius*unsafe_radius-(x-center_x)*(x-center_x));
        // Do this to fill the circle
        for(float i = center_y-y; i<=center_y+y; i+= node_distance) {
            if (isValidCoordinate(x, i)) {
                graph[coordToIndex(x)][coordToIndex(i)].closed = true;
                graph[coordToIndex(x)][coordToIndex(i)].unsafe = true;
            }
        }

    }
    // Do the same as over, just switch x and y
    for (float y = center_y-unsafe_radius; y <= center_y+unsafe_radius; y+=node_distance) {
        float x = sqrt(unsafe_radius*unsafe_radius-(y-center_y)*(y-center_y));
        for(float i = center_x-x; i<=center_x+x; i+= node_distance) {
            if (isValidCoordinate(i, y)) {
                graph[coordToIndex(i)][coordToIndex(y)].closed = true;
                graph[coordToIndex(i)][coordToIndex(y)].unsafe = true;
            }
        }
    }

}
void PathPlanner::refreshUnsafeZones(){
    std::cout << "refreshUnsafeZones" << std::endl;
    if(no_plan){
        return;
    }
    // delete all old unsafe zones
    for (float x = 0; x < FIELD_LENGTH; x += node_distance){
        for (float y = 0; y < FIELD_LENGTH; y += node_distance){
            graph[coordToIndex(x)][coordToIndex(y)].unsafe = false;
        }
    }


    for(auto current = obstacles.begin(); current != obstacles.end(); current++){
        addObstacle(current->x, current->y);
    }
}

float PathPlanner::calculateDiagonalHeuristic(float x, float y) {
    return (std::max(abs(x-end_node.getX()), abs(y-end_node.getY())));
}

float PathPlanner::calculateManhattanHeuristic(float x, float y){
    return (abs(x-end_node.getX()) + abs(y-end_node.getY()));
}

float PathPlanner::calculateEuclidianHeuristic(float x, float y) {
    return(sqrt((x-end_node.getX())*(x-end_node.getX())+(y-end_node.getY())*(y-end_node.getY())));
}

/*void PathPlanner::printGraph(){
    for(int i = 0; i < graph_size; i++){
        for(int j = 0; j < graph_size; j++){
            std::cout << std::fixed;
            std::cout << std::setprecision(2);
            std::cout << graph[i][j].getF() << "  ";
        }
        std::cout << std::endl;
    }
}*/

void PathPlanner::relaxGraph(){
    std::cout << "relaxGraph" << std::endl;
    Node current_node;
    while(!open_list.empty()){
        // Always search from the node with lowest f value
        current_node = open_list.top();
        open_list.pop();
        float x = current_node.getX();
        float y = current_node.getY();
        // Mark node as searched
        graph[coordToIndex(x)][coordToIndex(y)].closed = true;
        // Search all eight points around current point
        handleAllSuccessors(x,y);
        // Stop search if destination is reached
        if(destination_reached){return;}
    }
}

void PathPlanner::handleSuccessor(float x, float y, float parent_x, float parent_y, float distance_to_parent) {
    if(isValidCoordinate(x,y)){
        int x_index = coordToIndex(x);
        int y_index = coordToIndex(y);
        // Making sure the zero is actually zero and not something like 1.4895e-9
        if(x > 0.0 && x < 0.01){x = 0.0;}
        if(y > 0.0 && y < 0.01){y = 0.0;}
        if(isDestination(x,y) && !graph[x_index][y_index].closed){
            graph[x_index][y_index].setParentX(parent_x);
            graph[x_index][y_index].setParentY(parent_y);
            destination_reached = true;
            /*std::cout << "Destination reached: x=" << x << " y=" << y << std::endl;
            std::cout << "Destination parent: x=" << graph[x_index][y_index].getParentX()
                << " y=" << graph[coordToIndex(x)][coordToIndex(y)].getParentY() << std::endl;*/
        }
            // If node is not destination and hasn't been searched yet
        else if(!graph[x_index][y_index].closed){
            // Calculate distance from start through the parent
            float new_g = graph[coordToIndex(parent_x)][coordToIndex(parent_y)].getG() + distance_to_parent;
            // Update graph and open list if distance is less than before through the parent
            if(graph[x_index][y_index].getG() > new_g) {
                graph[x_index][y_index].updateF(new_g);
                // For visualization
                if(max_f < graph[x_index][y_index].getF()){
                    max_f = graph[x_index][y_index].getF();
                }
                graph[x_index][y_index].setParentX(parent_x);
                graph[x_index][y_index].setParentY(parent_y);
                open_list.push(graph[x_index][y_index]);
            }
        }
    }
}

void PathPlanner::handleAllSuccessors(float x, float y) {
    handleSuccessor(x+node_distance,y, x, y, node_distance);
    if(destination_reached){return;}
    handleSuccessor(x+node_distance, y+node_distance, x, y, diagonal_node_distance);
    if(destination_reached){return;}
    handleSuccessor(x+node_distance, y-node_distance, x, y, diagonal_node_distance);
    if(destination_reached){return;}
    handleSuccessor(x, y+node_distance, x, y, node_distance);
    if(destination_reached){return;}
    handleSuccessor(x, y-node_distance, x, y, node_distance);
    if(destination_reached){return;}
    handleSuccessor(x-node_distance, y, x, y, node_distance);
    if(destination_reached){return;}
    handleSuccessor(x-node_distance, y+node_distance, x, y, diagonal_node_distance);
    if(destination_reached){return;}
    handleSuccessor(x-node_distance, y-node_distance, x, y, diagonal_node_distance);
}

bool PathPlanner::isDestination(float x, float y) {
    return (coordToIndex(x) == coordToIndex(end_node.getX()) && coordToIndex(y) == coordToIndex(end_node.getY()));

}

bool PathPlanner::isStart(float x, float y) {
    return (coordToIndex(x) == coordToIndex(start_node.getX()) && coordToIndex(y) == coordToIndex(start_node.getY()));

}

bool PathPlanner::isValidCoordinate(float x, float y) {
    return(x>=0 && x<FIELD_LENGTH && y>=0 && y<FIELD_LENGTH);
}

bool PathPlanner::isSafeLine(float x1, float y1, float x2, float y2) {
    float delta_x = x2-x1;
    float delta_y = y2-y1;
    float num_points = 20*std::max(abs(delta_x),abs(delta_y));
    for(int i = 0; i < num_points; i++){
        x1 += delta_x/num_points;
        y1 += delta_y/num_points;
        // checking if the line goes through an obstacle at this point
        if(isValidCoordinate(x1,y1)){
            if (graph[coordToIndex(x1)][coordToIndex(y1)].obstacle) {
                return false;
            }
        }
    }
    return true;
}

bool PathPlanner::canSimplifyLine(float x1, float y1, float x2, float y2) {
    float delta_x = x2-x1;
    float delta_y = y2-y1;
    float num_points = 20*std::max(abs(delta_x),abs(delta_y));
    for(int i = 0; i < num_points; i++){
        x1 += delta_x/num_points;
        y1 += delta_y/num_points;
        // checking if the line goes through an obstacle at this point
        if(isValidCoordinate(x1,y1)){
            if (graph[coordToIndex(x1)][coordToIndex(y1)].unsafe) {
                return false;
            }
        }
    }
    return true;
}

Obstacle* PathPlanner::findBlockingObstacle(float current_x, float current_y) {
    std::cout << "Search for blocking obstacle" << std::endl;
    for (auto it = obstacles.begin(); it != obstacles.end(); it++) {
        if(abs(current_x-it->x) <= obstacle_radius*2 && abs(current_y-it->y) <= obstacle_radius*2) {
            std::cout << "Found blocking obstacle" << std::endl;
            return &(*it);
        }
    }
    std::cout << "Could not find blocking obstacle!!" << std::endl;
    return nullptr;
}

void PathPlanner::makePlan(float current_x, float current_y, float target_x, float target_y) {

    std::cout << "Run makePlan" << std::endl;

    no_plan = false;
    bool escape_from_obstacle = false;

    // If start or end point is not valid, a plan is not created
    if(!isValidCoordinate(current_x,current_y)){
        ROS_INFO("Current point invalid!");
        return;
    }
    if(!isValidCoordinate(target_x,target_y)){
        ROS_INFO("Target point invalid!");
        return;
    }

    resetParameters();

    start_node = Node(current_x, current_y, 0, 0);
    start_node.setParentX(0);
    start_node.setParentY(0);
    end_node = Node(target_x,target_y, std::numeric_limits<float>::infinity(), 0);
    initializeGraph();

    refreshObstacles();
    refreshUnsafeZones();


    if(graph[coordToIndex(current_x)][coordToIndex(current_y)].unsafe) {
        std::cout << "\n GET OOOOOUT!!! \n\n";
        escape_from_obstacle = true;

        // ---- make this a function ----

        // Figure out which obstacle is blocking
        Obstacle* blocking_obstacle = findBlockingObstacle(current_x, current_y);
        if (!blocking_obstacle) {
            std::cout << "Unable to solve problem!" << std::endl;
            return;
        }

        // Find the direction of the obstacle
        float obstacle_dir = blocking_obstacle->dir;
        
        // Find desired direction of the drone
        struct Point {float x,y;};
        Point current_point{current_x, current_y};
        float drone_dir = obstacle_math::calcAngleToObstacle(current_point, *blocking_obstacle, obstacle_dir);
       
        // Find setpoint in right direction outside unsafe zone
        // The safe zone is obstacle_radius*2, but to ensure that
        // the new point is outside the safe zone, 2.5 is used here
        float safe_radius = obstacle_radius*2.5;
        float safe_x = safe_radius*cos(drone_dir)+blocking_obstacle->x;
        float safe_y = safe_radius*sin(drone_dir)+blocking_obstacle->y;

        if(!isValidCoordinate(safe_x, safe_y)) {
            std::cout << "Cannot find safe point!" << std::endl;
            return;
        }

        // Add starting point to plan and plan a new plan from the
        // setpoint to target
        resetParameters();
        start_node = Node(safe_x, safe_y, 0, 0);
        start_node.setParentX(0);
        start_node.setParentY(0);
        initializeGraph();
        refreshObstacles();
        refreshUnsafeZones();
    }

    float x = end_node.getX(); 
    float y = end_node.getY();

    int x_index = coordToIndex(x);
    int y_index = coordToIndex(y);

    if(graph[x_index][y_index].unsafe){
        std::cout << "End point unsafe, invalid!" << std::endl;
        return;
    }


    // Calculate all f values and set the parents
    relaxGraph();

    // Dummy safety for avoiding infinite loop, will remove later
    int counter = 0;
    // Go through the graph from end to start through the parents of each node
    // Add nodes to the plan

    // Actual endpoint
    plan.push_front(end_node);

    while(!isStart(x, y)){
        // parent is calculated from the endpoint index, this node might have a different
        // coordinate (depending on the grid size)
        float x_temp = graph[x_index][y_index].getParentX();
        float y_temp = graph[x_index][y_index].getParentY();
        x = x_temp;
        y = y_temp;

        x_index = coordToIndex(x);
        y_index = coordToIndex(y);

        counter ++;
        if(counter >200){break;}
        plan.push_front(graph[x_index][y_index]);
    }

    

    // Print the plan - can be removed
    /*std::cout << std::endl << "Whole plan:" << std::endl;
    for (std::list<Node>::iterator it = plan.begin(); it!= plan.end(); ++it){
        std::cout << "x: " << it->getX() << " y: " << it->getY() << std::endl;
    }*/

    // Delete unnecessary points
    simplifyPlan();
    if(escape_from_obstacle) {
        simple_plan.push_front(graph[current_x][current_y]);
    }
}

void PathPlanner::simplifyPlan() {
    simple_plan = plan;
    // Pointing at the three first elements
    std::list<Node>::iterator first = simple_plan.begin();
    std::list<Node>::iterator second = ++(simple_plan.begin());
    std::list<Node>::iterator third = ++(simple_plan.begin());
    third++;

    if(first == simple_plan.end()){
        return;
    } else if(second == simple_plan.end()){
        return;
    }

    float x1, x2, y1, y2;

    while(third != simple_plan.end()){
        // Parameters for finding the straight line between the first and third point
        x1 = first->getX();
        x2 = third->getX();
        y1 = first->getY();
        y2 = third->getY();

        // If an unsafe zone is hit, the points are necessary and the search will start
        // at the end of the current search.
        if(!canSimplifyLine(x1,y1,x2,y2)){
            first++;
            second++;
            third++;
        }
            // If the straight line does not go through an unsafe zone, delete the second point
            // (no use in going through it when we can go in a straight line)
            // The search continues with same starting point, but the second and third point is changed
        else{
            second = simple_plan.erase(second);
            third++;
        }
    }

    // Print the remaining points
    /*std::cout << "Simple plan: " << std::endl;
    for(std::list<Node>::iterator it = simple_plan.begin(); it != simple_plan.end(); it++){
        std::cout << "x: " << it->getX() << " y: " << it->getY() << std::endl;
    }*/
}

bool PathPlanner::isPlanSafe(float current_x, float current_y) {
    if(simple_plan.empty()){
        return false;
    }

    std::list<Node>::iterator current = simple_plan.begin();
    std::list<Node>::iterator next = ++(simple_plan.begin());

    // Check if current point to first point in remaining plan is safe
    if(!isSafeLine(current_x, current_y, current->getX(), current->getY())){
        return false;
    }

    // Check if rest of plan is safe
    while(next != simple_plan.end() && current != simple_plan.end()){
        if(!isSafeLine(current->getX(), current->getY(), next->getX(), next->getY())){
            return false;
        }
        current++;
        next++;
    }
    return true;
}

void PathPlanner::resetParameters() {
    std::cout << "Reset Parameters" << std::endl;
    while(!open_list.empty()){
        open_list.pop();
    }
    plan.clear();
    simple_plan.clear();
    destination_reached = false;
}

bool PathPlanner::removeOldPoints(float current_x, float current_y){
    // Check if current point is the first point of the plan
    float margin = node_distance;

    if(simple_plan.empty()){
        return false;
    }

    // Do not remove end point from plan
    if(++simple_plan.begin() == simple_plan.end()){
        return false;
    }

    bool removed = false;
    if(abs(current_x-simple_plan.begin()->getX()) < margin && abs(current_y-simple_plan.begin()->getY())<margin){
        std::cout << "Remove " << simple_plan.begin()->getX() << ", "
                  << simple_plan.begin()->getY() << std::endl;
        simple_plan.pop_front();
        removed = true;
    }
    return removed;
}