#ifndef OBSTACLE_HANDLER_HPP
#define OBSTACLE_HANDLER_HPP
#include <ros/ros.h>
#include <ascend_msgs/GRState.h>
#include <ascend_msgs/GRStateArray.h>
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
    ascend_msgs::GRStateArray::ConstPtr last_msg_p_;    
    ///Callback to run when new message is recieved 
    void onMsgRecievedCB(ascend_msgs::GRStateArray::ConstPtr msg_p);
    ///Private constructor
    ObstacleStateHandler();
public:
    ///Get shared pointer to shared instance
    static const ObstacleStateHandler* getSharedObstacleHandlerPtr();
    ///Get last obstacles from shared instance
    static const std::vector<ascend_msgs::GRState>& getCurrentObstacles();
    ///Returns obstacles from instance
    const std::vector<ascend_msgs::GRState>& getObstacles() const;
    ///Is shared instance ready?
    static bool isInstanceReady();
    ///is current instance ready
    bool isReady() const;
};
}
#endif

