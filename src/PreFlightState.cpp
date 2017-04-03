#include "control_fsm/PreFlightState.hpp"
#include "control_fsm/ControlFSM.hpp"

//Only check for an abort event
void PreFlightState::handleEvent(ControlFSM& fsm, const EventData& event) {
	if(event.eventType == EventType::COMMAND) {
		event.eventError("Drone is not yet active (preflight) - command ignored");
		fsm.handleFSMWarn("Drone is not yet active - commands ignored");
	} else if(event.eventType == EventType::REQUEST) {
		if(event.request == RequestType::ABORT) {
			fsm.transitionTo(ControlFSM::BEGINSTATE, this, event);
		} else {
			fsm.handleFSMWarn("Invalid transition request");
		}
	} else if(event.eventType == EventType::ARMED) {
		fsm.transitionTo(ControlFSM::IDLESTATE, this, event); //Transition to IDLE when armed
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



