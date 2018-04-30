//
// Created by haavard on 19.02.18.
//

#ifndef CONTROL_FSM_GROUND_ROBOT_HANDLER_HPP
#define CONTROL_FSM_GROUND_ROBOT_HANDLER_HPP

#include <ros/ros.h>
#include <ascend_msgs/AIWorldObservation.h>

#include <utility>

namespace control {
class GroundRobotHandler {
    using MsgType = ascend_msgs::AIWorldObservation;
public:
    using GBVectorType = boost::array<ascend_msgs::GRState, 10>;
private:
    ///Shared instance
    static std::unique_ptr<GroundRobotHandler> shared_instance_p_;
    ///ROS nodehandle
    ros::NodeHandle n_;
    ///ROS subscriber to ground robot data
    ros::Subscriber gb_sub_;
    ///Last recieved message
    MsgType::ConstPtr last_gb_msg_p_;

    ///Callback for recieving ros data
    void gbCB(MsgType::ConstPtr msg_p) { last_gb_msg_p_ = std::move(msg_p); }
    ///Get valid shared instance
    static const GroundRobotHandler* getSharedInstancePtr();
    ///Constructor
    GroundRobotHandler();
    ///Get ground robots from instance
    const GBVectorType& getGroundRobots() const;

public:
    static GBVectorType getCurrentGroundRobots() { return getSharedInstancePtr()->getGroundRobots(); }
    ///Check if ready
    static bool isReady();



};
}

#endif //CONTROL_FSM_GROUND_ROBOT_HANDLER_HPP
