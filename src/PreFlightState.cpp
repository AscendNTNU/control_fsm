#include "PreFlightState.hpp"
#include "ControlFSM.hpp"

//Only check for an abort event
void PreFlightState::handleEvent(ControlFSM& fsm, const EventData& event) {
	if(event.request == RequestType::ABORT) {
		//Transition back to start
		fsm.transitionTo(ControlFSM::BEGINSTATE, this);
	} else {
		fsm.handleFSMInfo("Invalid transiton");
	}
}

//Not doing much - just prints a current state for debugging purposes
void PreFlightState::stateBegin(ControlFSM& fsm, const EventData& event) {
	fsm.handleFSMInfo("Current state: Preflight");
}

//Returns setpoint
const mavros_msgs::PositionTarget& PreFlightState::getSetpoint() const { return _setpoint; }



