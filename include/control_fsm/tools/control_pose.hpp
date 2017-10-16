#ifndef CONTROL_POSE
#define CONTROL_POSE
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <array>
#include <std_msgs/Header.h>

namespace control {
struct Point {
    double x;
    double y;
    double z;

    Point(double x, double y, double z) : x(x), y(y), z(z) {}

    Point() : x(), y(), z() {}
};


struct Quaternion {
    double x;
    double y;
    double z;
    double w;
};


//Forward declaration
class Pose;

/**
 * Handles drones pose
 * Uses shared_ptr to the same memory to avoid duplicates.
 */
class Pose {
private:
    ///One static shared_ptr instance
    static std::shared_ptr<Pose> instance_;

    ///Nodehandle
    ros::NodeHandle n_;
    ///Last recieved position
    geometry_msgs::PoseStamped last_position_;
    ///Subscriber - subscribes to mavros position topic
    ros::Subscriber pos_sub_;

    ///Callback for new position messages
    void positionCB(const geometry_msgs::PoseStamped &msg) { last_position_ = msg; }

    ///Constructor - only accessible from static functions
    Pose();

public:
    ///Returns array of position in order x, y, z
    control::Point getPositionXYZ();

    ///Returns yaw value in radians with negative PI/2 offsett to correct mavros bug.
    double getMavrosCorrectedYaw();

    ///Returns raw yaw calculated from orientation quaternions
    double getYaw();

    ///Returns raw orientation quaternion
    Quaternion getOrientation();

    ///Get header - includes timestamp
    std_msgs::Header getHeader() { return last_position_.header; }

    ///Returns pointer to instance
    static std::shared_ptr<Pose> getSharedPosePtr();

    ///Check if pose is recieved
    bool isPoseValid();

};
}
#endif
