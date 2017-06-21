#include "control_fsm/TrackGBState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>

TrackGBState::TrackGBState() {
    //TODO Set correct setpoint type here (default might be correct?)
}

void TrackGBState::handleEvent(ControlFSM& fsm, const EventData& event) {
    //TODO Handle all transition requests
}

void TrackGBState::stateBegin(ControlFSM& fsm, const EventData& event) {
    //TODO Implement
}

void TrackGBState::loopState(ControlFSM& fsm) {
    //TODO Implement ground robot tracking
}

const mavros_msgs::PositionTarget* TrackGBState::getSetpoint() {
    _setpoint.header.stamp = ros::Time::now();
    return &_setpoint;
}