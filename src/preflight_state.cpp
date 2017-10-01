#include "control_fsm/preflight_state.hpp"
#include "control_fsm/control_fsm.hpp"
#include "control_fsm/setpoint_msg_defines.h"

PreFlightState::PreFlightState() {
    _setpoint.type_mask = default_mask | SETPOINT_TYPE_IDLE;
}
//Only check for an abort event
void PreFlightState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidCMD()) {
        event.eventError("CMD rejected!");
        fsm.handleFSMWarn("Drone is not yet active - commands ignored");
    } else if(event.isValidRequest()) {
        if(event.request == RequestType::ABORT) {
            fsm.handleFSMWarn("Can't abort preflight!");
        } else if(event.request == RequestType::MANUALFLIGHT) {
            fsm.transitionTo(ControlFSM::MANUALFLIGHTSTATE, this, event);
        } else {
            fsm.handleFSMWarn("Invalid transition request");
        }
    } else if(event.eventType == EventType::AUTONOMOUS) {
        fsm.transitionTo(ControlFSM::IDLESTATE, this, event); //Transition to IDLE when armed and ready
    } else {
        fsm.handleFSMInfo("Event ignored");
    }
}

void PreFlightState::stateBegin(ControlFSM &fsm, const EventData &event) {
    fsm.handleFSMInfo("Preflight mode: Arm and set OFFBOARD to proceed to IDLE!");
}

//Returns setpoint
const mavros_msgs::PositionTarget* PreFlightState::getSetpoint() { 
    _setpoint.header.stamp = ros::Time::now();
    return &_setpoint; 
}

void PreFlightState::handleManual(ControlFSM &fsm) {
    //Do nothing
}



