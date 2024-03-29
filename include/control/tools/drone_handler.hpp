#ifndef CONTROL_DRONE_HANDLER_HPP
#define CONTROL_DRONE_HANDLER_HPP
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/State.h>
#include <memory>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
namespace control {
/** Handles state information recieved from the drone
 * Singleton pattern gurantees one, and only one instance
 * will always exists. Returning references are therefore safe
 **/
class DroneHandler;
class DroneHandler {
private:
    ///Shared instance - will always exist after created!
    static std::unique_ptr<DroneHandler> shared_instance_p_;

    ///Transform buffer
    tf2_ros::Buffer tf_buffer_;
    ///Transform listener
    tf2_ros::TransformListener tf_listener_;
    ///Nodehandler
    ros::NodeHandle n_;
    ///Subscriber recieving pose
    ros::Subscriber pose_sub_;
    ///Subscriber recieving velocity
    ros::Subscriber twist_sub_;
    ///Subscriber for recieving uncorrected altitude from lidar
    ros::Subscriber raw_dist_sub_;
    ///Holds last recieved pose
    geometry_msgs::PoseStamped::ConstPtr last_pose_;
    ///Holds last recieved twist
    geometry_msgs::TwistStamped::ConstPtr last_twist_;
    ///Holds last recieved distance
    sensor_msgs::Range::ConstPtr last_dist_;
    

    ///Private constructor
    DroneHandler();
    ///On new pose recieved
    void onPoseRecievedCB(geometry_msgs::PoseStamped::ConstPtr msg_p) { last_pose_ = msg_p; }
    ///On new twist recieved
    void onTwistRecievedCB(geometry_msgs::TwistStamped::ConstPtr msg_p) { last_twist_ = msg_p;}
    
    void onDistRecievedCB(sensor_msgs::Range::ConstPtr msg_p) { last_dist_ = msg_p; }
    ///Get latest local pose
    const geometry_msgs::PoseStamped& getLocalPose() const;
    ///Get latest twist
    const geometry_msgs::TwistStamped& getTwist() const;
    ///Get latest distance
    const sensor_msgs::Range& getDistance() const;
    ///Returns pointer to shared instance
    static const DroneHandler* getSharedInstancePtr();

public:
    ///Returns last local pose from shared instance
    static geometry_msgs::PoseStamped getCurrentLocalPose();
    ///Returns last global pose from shared instance
    static geometry_msgs::PoseStamped getCurrentGlobalPose();
    ///Returns last twist from shared instance
    static geometry_msgs::TwistStamped getCurrentTwist();
    ///Returns last distance
    static sensor_msgs::Range getCurrentDistance();
    ///Is pose data valid?
    static bool isLocalPoseValid();
    ///Is twist data valid?
    static bool isTwistValid();
    ///Is pose data and transform valid
    static bool isGlobalPoseValid();
    ///Is transform data valid
    static bool isTransformsValid();
    ///Is distance data valid 
    static bool isDistValid();
    ///Get transform from global to local frame
    static geometry_msgs::TransformStamped getGlobal2LocalTf();
    ///Get transform from local to global frame
    static geometry_msgs::TransformStamped getLocal2GlobalTf();
};
}
#endif

