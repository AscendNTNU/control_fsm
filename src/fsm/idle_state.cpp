#include "control/fsm/idle_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <ros/ros.h>
#include <control/tools/logger.hpp>
#include "control/fsm/event_data.hpp"
#include "control/fsm/control_fsm.hpp"

//Sets setpoint type to IDLE
IdleState::IdleState() {
    setpoint_ = mavros_msgs::PositionTarget();
    setpoint_.type_mask = default_mask          |
                          SETPOINT_TYPE_IDLE    |
                          IGNORE_PX             |
                          IGNORE_PY             |
                          IGNORE_PZ             |
                          IGNORE_YAW;
}

void IdleState::handleEvent(ControlFSM& fsm, const EventData& event) {
    //All commands needs to get to position hold first
    if(event.isValidRequest()) {
        if(event.request == RequestType::TAKEOFF) {
            fsm.transitionTo(ControlFSM::TAKEOFF_STATE, this, event);
        } else {
            control::handleWarnMsg("Invalid transition request");
        }
    } else if(event.isValidCMD()) {
        fsm.transitionTo(ControlFSM::TAKEOFF_STATE, this, event);
    } else  {
        control::handleInfoMsg("Event ignored");
    }
}

const mavros_msgs::PositionTarget* IdleState::getSetpointPtr() {
    //Sets timestamp, and returns setpoint_ as const pointer
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

void IdleState::handleManual(ControlFSM &fsm) {
    RequestEvent manual_event(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUAL_FLIGHT_STATE, this, manual_event);
}



ascend_msgs::ControlFSMState IdleState::getStateMsg() {
    using ascend_msgs::ControlFSMState;
    ControlFSMState msg;
    msg.name = getStateName();
    msg.state_id = ControlFSMState::IDLE_STATE;
    return msg;
}
