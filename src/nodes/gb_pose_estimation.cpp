#include <ros/ros.h>
#include "control/tools/ground_robot_handler.hpp"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "control/tools/gb_estimator.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <control/tools/control_message.hpp>
#include <ascend_msgs/GroundRobotTransform.h>

enum class State { LOCAL_POSITION, GB_POSITION };

State current_state = State::LOCAL_POSITION;
int target_gb = -1;

ros::Publisher filter_pub;

using ascend_msgs::AIWorldObservation;
using ascend_msgs::GroundRobotTransform;

using geometry_msgs::PoseStamped;

geometry_msgs::PoseStamped::ConstPtr last_pose_p;

void localPositionCB(PoseStamped::ConstPtr msg_p);
void groundRobotCB(const AIWorldObservation& msg);
bool serviceCB(GroundRobotTransform::Request& req,
               GroundRobotTransform::Response& resp);

control::GbEstimator gb_est;



int main(int argc, char **argv){
    //Initalize ros node
    ros::init(argc, argv, "gb_pose_estimation_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");
    
    std::string gb_topic = "/detectedGroundRobots";
    std::string local_pos_topic = "/perception/local_position";
    std::string filter_topic = "/local_position";
    
    if(!np.getParam("gb_topic", gb_topic)) {
        ROS_WARN("Using gb_topic = %s", gb_topic.c_str());
    }
    if(!np.getParam("local_pos_topic", local_pos_topic)) {
        ROS_WARN("Using local_pos_topic = %s", local_pos_topic.c_str());
    }
    
    if(!np.getParam("filter_topic", filter_topic)) {
        ROS_WARN("Using filter_topic = %s", filter_topic.c_str());
    }
    ros::Subscriber local_position_sub = n.subscribe(local_pos_topic, 1, localPositionCB);
    ros::Subscriber ground_robot_sub = n.subscribe(gb_topic, 1, groundRobotCB);
    ros::ServiceServer service = n.advertiseService("/control/pose/gb_estimation", serviceCB);
    filter_pub = n.advertise<PoseStamped>(filter_topic, 1);
    
    ros::spin();
}

bool serviceCB(GroundRobotTransform::Request& req,
               GroundRobotTransform::Response& resp) {
    if(req.state == req.GBFRAME) {
        target_gb = req.gb_id;
        current_state = State::GB_POSITION;
    } else if(req.state == req.LOCALFRAME) {
        target_gb = -1;
        current_state = State::LOCAL_POSITION;
    } else {
        ROS_ERROR_NAMED("GB estimation", "Invalid requested state");
    }
    resp.success = static_cast<unsigned char>(true);
    return true;
}

void localPositionCB(PoseStamped::ConstPtr msg_p) {
    if(current_state == State::LOCAL_POSITION) {
        filter_pub.publish(*msg_p);
    }
    last_pose_p = std::move(msg_p);
}

void groundRobotCB(const AIWorldObservation& msg) {
    //Wait for pose before sending
    if(last_pose_p == nullptr) return;
    if(target_gb >= 0 && target_gb < static_cast<int>(msg.ground_robots.size())) return;

    if(current_state == State::GB_POSITION) {
        auto& gb = msg.ground_robots[target_gb];

        std::array<float, 2> dir_vec{ std::cos(gb.theta), std::sin(gb.theta)};
        std::array<float, 2> pos_vec{};

        gb_est.update(dir_vec.data(), pos_vec.data());

        const double delta_x = gb.x - msg.drone_position.x;
        const double delta_y = gb.y - msg.drone_position.y;

        const double drone_x = pos_vec[0] - delta_x;
        const double drone_y = pos_vec[1] - delta_y;

        if(control::message::hasTimedOut(*last_pose_p)) {
            ROS_ERROR_NAMED("GB estimation", "Data timeout");
            return;
        }

        PoseStamped pub_msg;
        pub_msg = *last_pose_p;
        pub_msg.pose.position.x = drone_x;
        pub_msg.pose.position.y = drone_y;
        pub_msg.header.stamp = ros::Time::now();
        filter_pub.publish(pub_msg);
    }

}
