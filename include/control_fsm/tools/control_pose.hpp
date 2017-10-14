#ifndef CONTROL_POSE
#define CONTROL_POSE
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <array>
#include <std_msgs/Header.h>
class ControlPose {
private:
    ///Nodehandle
    ros::NodeHandle n_;
    ///Last recieved position
    geometry_msgs::PoseStamped last_position_;
    ///Subscriber - subscribes to mavros position topic
    ros::Subscriber pos_sub_;
    ///Callback for new position messages
    void positionCB(const geometry_msgs::PoseStamped& msg) { last_position_ = msg; }

public:

    ///Constructor
    ControlPose();
    ///Returns array of position in order x, y, z
    std::array<float, 3> get_position_xyz();
    ///Returns yaw value in radians with negative PI/2 offsett to correct mavros bug.
    float get_mavros_corrected_yaw();
    ///Returns raw yaw calculated from orientation quaternions
    float get_yaw();
    ///Returns raw orientation quaternion
    std::array<float, 4> get_orientation();
    ///Get header - includes timestamp
    std_msgs::Header get_header();

};

#endif