#include <ros/ros.h>
#include "control/tools/perception_validation.hpp"
#include <geometry_msgs/Vector3Stamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "validation");
    ros::NodeHandle n;
    
    using control::validation::PoseStampedCorrelation;
    ros::Publisher pub = n.advertise<geometry_msgs::Vector3Stamped>("/validation", 1);
    
    PoseStampedCorrelation corr_calc("/mavros/mocap/pose", "/mavros/local_position/pose");
    ros::Duration(2.0).sleep();
    corr_calc.start();
    ros::spin();
}
