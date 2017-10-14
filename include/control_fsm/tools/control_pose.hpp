#ifndef CONTROL_POSE
#define CONTROL_POSE
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <array>
#include <std_msgs/Header.h>
//Forward declaration
class ControlPose;

/**
 * Handles drones pose
 * Uses shared_ptr to the same memory to avoid duplicates.
 */
class ControlPose {
private:
    ///One static shared_ptr instance
    static std::shared_ptr<ControlPose> instance_;

    ///Nodehandle
    ros::NodeHandle n_;
    ///Last recieved position
    geometry_msgs::PoseStamped last_position_;
    ///Subscriber - subscribes to mavros position topic
    ros::Subscriber pos_sub_;
    ///Callback for new position messages
    void positionCB(const geometry_msgs::PoseStamped& msg) { last_position_ = msg; }
    ///Constructor - only accessible from static functions
    ControlPose();
public:
    ///Returns array of position in order x, y, z
    std::array<float, 3> getPositionXYZ();
    ///Returns yaw value in radians with negative PI/2 offsett to correct mavros bug.
    float getMavrosCorrectedYaw();
    ///Returns raw yaw calculated from orientation quaternions
    float getYaw();
    ///Returns raw orientation quaternion
    std::array<float, 4> getOrientation();
    ///Get header - includes timestamp
    std_msgs::Header getHeader() { return last_position_.header; }

    ///Returns pointer to instance
    static std::shared_ptr<ControlPose> getSharedPosePtr();
    ///Check if pose is recieved
    bool isPoseValid();

};

#endif
