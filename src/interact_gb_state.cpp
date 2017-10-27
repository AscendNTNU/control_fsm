#include "control_fsm/interact_gb_state.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>

InteractGBState::InteractGBState() {
    //TODO Implement correct setpoint type
}

void InteractGBState::handleEvent(ControlFSM& fsm, const EventData& event) {
    //TODO Handle all transition requests
}

void InteractGBState::stateBegin(ControlFSM& fsm, const EventData& event) {
    //TODO Implement
}

void InteractGBState::loopState(ControlFSM& fsm) {
    //TODO Implement ground robot tracking
}

const mavros_msgs::PositionTarget* InteractGBState::getSetpoint() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

void InteractGBState::handleManual(ControlFSM &fsm) {
    //TODO Implement
}