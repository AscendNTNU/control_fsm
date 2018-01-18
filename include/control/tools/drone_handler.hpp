#ifndef CONTROL_DRONE_HANDLER_HPP
#define CONTROL_DRONE_HANDLER_HPP
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
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
    ///Subscriber recieving velocity
    ros::Subscriber twist_sub_;
    ///Holds last recieved pose
    geometry_msgs::PoseStamped::ConstPtr last_pose_;
    ///Holds last recieved twist
    geometry_msgs::TwistStamped::ConstPtr last_twist_;
    ///Private constructor
    DroneHandler();
    ///On new pose recieved
    void onPoseRecievedCB(geometry_msgs::PoseStamped::ConstPtr msg_p) { last_pose_ = msg_p; }
    ///On new twist recieved
    void onTwistRecievedCB(geometry_msgs::TwistStamped::ConstPtr msg_p) { last_twist_ = msg_p;}
public:
    ///Returns pointer to shared instance
    static std::shared_ptr<DroneHandler> getSharedInstancePtr();
    ///Returns last pose from shared instance
    static const geometry_msgs::PoseStamped& getCurrentPose();
    ///Returns last twist from shared instance
    static const geometry_msgs::TwistStamped& getCurrentTwist();
    ///Get latest pose
    const geometry_msgs::PoseStamped& getPose() const;
    ///Get latest twist
    const geometry_msgs::TwistStamped& getTwist() const;
    ///Is pose data valid?
    static bool isPoseValid();
    ///Is twist data valid?
    static bool isTwistValid();
};
}
#endif

