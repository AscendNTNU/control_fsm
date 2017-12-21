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
    ///Shared instance
    static std::shared_ptr<DroneHandler> shared_instance_p_;
    ///Nodehandler
    ros::NodeHandle n_;
    ///Subscriber recieving pose
    ros::Subscriber pose_sub_;
    ///Subscriber recieving state 
    ros::Subscriber state_sub_;
    ///Holds last recieved pose
    geometry_msgs::PoseStamped last_pose_;
    ///Holds last recieved state
    mavros_msgs::State last_state_;
    ///Private constructor
    DroneHandler();
    ///On new state recieved
    void onStateRecievedCB(const mavros_msgs::State& msg) { last_state_ = msg; }
    ///On new pose recieved
    void onPoseRecievedCB(const geometry_msgs::PoseStamped& msg) { last_pose_ = msg; }
public:
    ///Returns pointer to shared instance
    static std::shared_ptr<DroneHandler> getSharedInstancePtr();
    ///Returns last pose from shared instance
    static const geometry_msgs::PoseStamped& getCurrentPose();
    ///Returns last state from shared instance
    static const mavros_msgs::State& getCurrentState();
    ///Get latest pose
    const geometry_msgs::PoseStamped& getPose() const;
    ///Get latest state
    const mavros_msgs::State& getState() const;
    ///Is pose data valid?
    static bool isPoseValid();
    ///Is state data valid?
    static bool isStateValid();
};
}
#endif

