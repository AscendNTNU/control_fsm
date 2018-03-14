#include "control/fsm/interact_gb_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
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

const mavros_msgs::PositionTarget* InteractGBState::getSetpointPtr() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

void InteractGBState::handleManual(ControlFSM &fsm) {
    //TODO Implement
}


ascend_msgs::ControlFSMState InteractGBState::getStateMsg() const {
    using ascend_msgs::ControlFSMState;
    ControlFSMState msg;
    msg.name = getStateName();
    msg.state_id = ControlFSMState::INTERACT_GB_STATE;
    return msg;
}
