#include "control_fsm/PreFlightState.hpp"
#include "control_fsm/ControlFSM.hpp"
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
			fsm.transitionTo(ControlFSM::BEGINSTATE, this, event);
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


//Returns setpoint
const mavros_msgs::PositionTarget* PreFlightState::getSetpoint() { 
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint; 
}



