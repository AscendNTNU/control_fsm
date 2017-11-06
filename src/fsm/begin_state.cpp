#include "control/fsm/begin_state.hpp"
#include "control/fsm/control_fsm.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <ros/ros.h>
#include <control/tools/logger.hpp>

BeginState::BeginState() {
    setpoint_.type_mask = default_mask | SETPOINT_TYPE_IDLE;
}    

//Begin state only waits for preflight request
void BeginState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidRequest()) {
        if(event.request == RequestType::PREFLIGHT) {
            fsm.transitionTo(ControlFSM::PREFLIGHT_STATE, this, event);
        } else {
            control::handleWarnMsg("Invalid transiton request!");
        }
    } else if(event.isValidCMD()) {
        event.eventError("CMD rejected!");
        control::handleWarnMsg("Drone is not yet active - commands ignored");
    } else {
        control::handleInfoMsg("Event ignored!");
    }
}

//Returns IDLE setpoints - nothing will though happen as drone should be disarmed
const mavros_msgs::PositionTarget* BeginState::getSetpointPtr() {
    setpoint_.header.stamp = ros::Time::now();
     return &setpoint_;
}

void BeginState::handleManual(ControlFSM &fsm) {
    //Does nothing
}



