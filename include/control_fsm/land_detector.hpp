//
// Created by haavard on 01.06.17.
//

#ifndef CONTROL_FSM_LANDDETECTOR_HPP
#define CONTROL_FSM_LANDDETECTOR_HPP

#include <ros/ros.h>
#include <ascend_msgs/BoolStamped.h>

class ControlFSM;

class LandDetector {

private:
    std::string topic_;
    ControlFSM* p_fsm_ = nullptr;
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ascend_msgs::BoolStamped last_msg_;
    void landCB(const ascend_msgs::BoolStamped& msg);

public:
    LandDetector(std::string topic);
    LandDetector(std::string topic, ControlFSM* p_fsm);
    bool isOnGround();
    bool isReady();
};

#endif //CONTROL_FSM_LANDDETECTOR_HPP
