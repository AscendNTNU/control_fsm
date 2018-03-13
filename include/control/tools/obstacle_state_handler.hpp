#ifndef OBSTACLE_HANDLER_HPP
#define OBSTACLE_HANDLER_HPP
#include <ros/ros.h>
#include <ascend_msgs/DetectedRobotsGlobalPositions.h>
#include "control/tools/config.hpp"
#include "control/tools/logger.hpp"
#include "control_message.hpp"

namespace control {
class ObstacleStateHandler {
private:
    ///Shared instance
    static std::unique_ptr<ObstacleStateHandler> shared_instance_p_;
    ///ROS nodehanlder
    ros::NodeHandle n_;
    ///Subscriber to obstacle stream
    ros::Subscriber obs_sub_;
    ///Last recieved message
    ascend_msgs::DetectedRobotsGlobalPositions::ConstPtr last_msg_p_;    
    ///Callback to run when new message is recieved 
    void onMsgRecievedCB(ascend_msgs::DetectedRobotsGlobalPositions::ConstPtr msg_p);
    ///Private constructor
    ObstacleStateHandler();
    ///Get shared pointer to shared instance
    static const ObstacleStateHandler* getSharedObstacleHandlerPtr();
    ///Returns obstacles from instance
    const ascend_msgs::DetectedRobotsGlobalPositions& getObstacles() const;
    ///is current instance ready
    bool isReady() const;
public:
    ///Get last obstacles from shared instance
    static const ascend_msgs::DetectedRobotsGlobalPositions& getCurrentObstacles();
    ///Is shared instance ready?
    static bool isInstanceReady();
};
}
#endif

