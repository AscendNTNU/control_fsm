//
// Created by haavard on 01.06.17.
//

#ifndef CONTROL_FSM_LANDDETECTOR_HPP
#define CONTROL_FSM_LANDDETECTOR_HPP

#include <ros/ros.h>
#include <mavros_msgs/ExtendedState.h>

class ControlFSM;

class LandDetector {

private:
    std::string _topic;
    ControlFSM* _pFsm = nullptr;
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    mavros_msgs::ExtendedState _lastMsg;
    void landCB(const mavros_msgs::ExtendedState& msg);

public:
    LandDetector(std::string topic);
    LandDetector(std::string topic, ControlFSM* _pFsm);
    bool isOnGround();
    bool isReady();
};

#endif //CONTROL_FSM_LANDDETECTOR_HPP
