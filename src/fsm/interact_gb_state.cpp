#include "control/fsm/interact_gb_state.hpp"
#include "control/fsm/control_fsm.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <ros/ros.h>

InteractGBState::InteractGBState() {
    //TODO Implement correct setpoint type
}

void InteractGBState::handleEvent(ControlFSM& fsm, const EventData& event) {
    //TODO Handle all transition requests
}

void InteractGBState::stateBegin(ControlFSM& fsm, const EventData& event) {

    if (true /*TODO:check with obstacle avoidance*/){
        fsm.obstacle_avoidance_.relaxResponsibility();
    } else{
        RequestEvent abort_event(RequestType::ABORT);
        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
        return;
    }
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
