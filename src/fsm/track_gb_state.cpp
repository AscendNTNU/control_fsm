#include "control/fsm/track_gb_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
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

const mavros_msgs::PositionTarget* TrackGBState::getSetpointPtr() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

void TrackGBState::handleManual(ControlFSM &fsm) {
    //TODO Implement
}


ascend_msgs::ControlFSMState TrackGBState::getStateMsg() {
    using ascend_msgs::ControlFSMState;
    ControlFSMState msg;
    msg.name = getStateName();
    msg.state_id = ControlFSMState::TRACK_GB_STATE;
    return msg;
}
