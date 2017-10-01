#include "control_fsm/interact_gb_state.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>

InteractGBState::InteractGBState() {
    //TODO Implement correct setpoint type
}

void InteractGBState::handleEvent(ControlFSM& fsm, const event_data& event) {
    //TODO Handle all transition requests
}

void InteractGBState::stateBegin(ControlFSM& fsm, const event_data& event) {
    //TODO Implement
}

void InteractGBState::loopState(ControlFSM& fsm) {
    //TODO Implement ground robot tracking
}

const mavros_msgs::PositionTarget* InteractGBState::getSetpoint() {
    _setpoint.header.stamp = ros::Time::now();
    return &_setpoint;
}

void InteractGBState::handleManual(ControlFSM &fsm) {
    //TODO Implement
}
