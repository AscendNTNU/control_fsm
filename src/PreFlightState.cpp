#include "control_fsm/PreFlightState.hpp"
#include "control_fsm/ControlFSM.hpp"
#include "control_fsm/setpoint_msg_defines.h"

PreFlightState::PreFlightState() {
	_setpoint.type_mask = default_mask | SETPOINT_TYPE_IDLE;
}
//Only check for an abort event
void PreFlightState::handleEvent(ControlFSM& fsm, const EventData& event) {
	if(event.isValidCMD()) {
		handleCMD(fsm, event);
	} else if(event.isValidRequest()) {
		if(event.request == RequestType::ABORT) {
			handleAbort(fsm);
		} else if(event.request == RequestType::MANUALFLIGHT) {
			fsm.transitionTo(ControlFSM::MANUALFLIGHTSTATE, this, event);
		} else {
			fsm.handleFSMWarn("Invalid transition request");
		}
	} else if(event.eventType == EventType::AUTONOMOUS) {
		fsm.transitionTo(ControlFSM::IDLESTATE, this, event); //Transition to IDLE when armed and ready
		fsm._isActive = true;
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

void PreFlightState::handleAbort(ControlFSM &fsm) {
	fsm.handleFSMWarn("Nothing to abort!");
}

void PreFlightState::handleCMD(ControlFSM &fsm, const EventData &event) {
	if(event.isValidCMD()) {
		event.eventError("CMD rejected!");
		fsm.handleFSMWarn("Not accepting commands in preflight!");
	} else {
		fsm.handleFSMError("Invalid CMD!");
	}
}



