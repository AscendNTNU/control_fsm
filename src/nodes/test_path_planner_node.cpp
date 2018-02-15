#include <ros/ros.h>



int main(int argc, char** argv){

	ros::init(argc, argv, "path_planner_test");
    ros::NodeHandle n;
    ros::Rate rate(30.0);

    ros::Publisher pub_goal = n.advertise<>();
}