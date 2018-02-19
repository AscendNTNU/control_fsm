#include <control/tools/logger.hpp>
#include "control/fsm/preflight_state.hpp"
#include "control/fsm/control_fsm.hpp"
#include "control/tools/setpoint_msg_defines.h"

PreFlightState::PreFlightState() {
    setpoint_.type_mask = default_mask | SETPOINT_TYPE_IDLE;
}
//Only check for an abort event
void PreFlightState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidRequest()) {
        if(event.request == RequestType::ABORT) {
            control::handleWarnMsg("Can't abort preflight!");
        } else if(event.request == RequestType::MANUALFLIGHT) {
            fsm.transitionTo(ControlFSM::MANUAL_FLIGHT_STATE, this, event);
        } else {
            control::handleWarnMsg("Invalid transition request");
        }
    } else if(event.isValidCMD()) {
        event.eventError("CMD rejected!");
        control::handleWarnMsg("Drone is not yet active - commands ignored");
    } else if(event.event_type == EventType::AUTONOMOUS) {
        fsm.transitionTo(ControlFSM::IDLE_STATE, this, event); //Transition to IDLE when armed and ready
    } else {
        control::handleInfoMsg("Event ignored");
    }
}

void PreFlightState::stateBegin(ControlFSM &fsm, const EventData &event) {
    control::handleInfoMsg("Preflight mode: Arm and set OFFBOARD to proceed to IDLE!");
}

//Returns setpoint
const mavros_msgs::PositionTarget* PreFlightState::getSetpointPtr() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

void PreFlightState::handleManual(ControlFSM &fsm) {
    //Do nothing
}



