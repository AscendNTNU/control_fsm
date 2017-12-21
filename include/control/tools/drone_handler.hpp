#ifndef CONTROL_DRONE_HANDLER_HPP
#define CONTROL_DRONE_HANDLER_HPP
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <memory>
#include <ros/ros.h>
namespace control {
/** Handles state information recieved from the drone
 * Singleton pattern gurantees one, and only one instance
 * will always exists. Returning references are therefore safe
 **/
class DroneHandler;
class DroneHandler {
private:
    ///Shared instance - will always exist after created!
    static std::shared_ptr<DroneHandler> shared_instance_p_;
    ///Nodehandler
    ros::NodeHandle n_;
    ///Subscriber recieving pose
    ros::Subscriber pose_sub_;
    ///Holds last recieved pose
    geometry_msgs::PoseStamped last_pose_;
    ///Private constructor
    DroneHandler();
    ///On new pose recieved
    void onPoseRecievedCB(const geometry_msgs::PoseStamped& msg) { last_pose_ = msg; }
public:
    ///Returns pointer to shared instance
    static std::shared_ptr<DroneHandler> getSharedInstancePtr();
    ///Returns last pose from shared instance
    static const geometry_msgs::PoseStamped& getCurrentPose();
    ///Get latest pose
    const geometry_msgs::PoseStamped& getPose() const;
    ///Is pose data valid?
    static bool isPoseValid();
};
}
#endif

