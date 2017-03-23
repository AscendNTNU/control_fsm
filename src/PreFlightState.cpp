#include "PreFlightState.hpp"
#include "ControlFSM.hpp"

void PreFlightState::handleEvent(ControlFSM& fsm, const EventData& event) {
	if(event.request == RequestType::ABORT) {
		//Transition back to start
		fsm.transitionTo(ControlFSM::BEGINSTATE, this);
	} else {
		fsm.handleFSMInfo("Invalid transiton");
	}
}

void PreFlightState::stateBegin(ControlFSM& fsm, const EventData& event) {
	//Nothing to do here really
}

void PreFlightState::loopState(ControlFSM& fsm) {
	/*
	TODO Do preflight here
	After preflight -> transition to idle
	*/

}



