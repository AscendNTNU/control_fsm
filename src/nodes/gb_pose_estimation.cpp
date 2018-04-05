#include <ros/ros.h>
#include "control/tools/ground_robot_handler.hpp"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "GbEstimator.h"

class enum State { LOCAL_POSITION, GB_POSITION };

State current_state = State::INIT;

ros::Subscriber local_position_sub;
ros::Subscriber ground_robot_sub;
ros::Publisher filter_pub;

using geometry_msgs::PoseWithCovarianceStamped;
using ascend_msgs::AIWorldObservation;

void localPositionCB(PoseWithCovarianceStamped::ConstPtr msg_p);
void groundRobotCB(AIWorldObservation::ConstPtr msg_p);

GbEstimator gb_est;




int main(int argc, char **argv){
    //Initalize ros node
    ros::init(argc, argv, "gb_pose_estimation_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");
    
    std::string gb_topic = "/detectedGroundRobots";
    std::string local_pos_topic = "/perception/local_position";
    std::string filter_topic = "/local_position";
    
    if(!np.getParam("gb_topic", gb_topic)) {
        ROS_WARN("Using gb_topic = %s", gb_topic);
    }
    if(!np.getParam("local_pos_topic", local_pos_topic)) {
        ROS_WARN("Using local_pos_topic = %s", local_pos_topic);
    }
    
    if(!np.getParam("filter_topic", filter_topic)) {
        ROS_WARN("Using filter_topic = %s", filter_topic);
    }
    local_position_sub = n.subscribe(local_pos_topic.c_str(), 1, localPositionCB);
    ground_robot_sub = n.subscribe(gb_topic.c_str(), 1, groundRobotCB);
    filter_pub = n.advertise<PoseWithCovarianceStamped>(filter_topic.c_str(), 1);
    
    ros::spin();
}

void localPositionCB(const PoseWithCovarianceStamped& msg) {
    if(current_state == State::LOCAL_POSITION) {
        filter_pub.publish(msg);
    }
}

void groundRobotCB(const AIWorldObservation& msg) {
    

}
