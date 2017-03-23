#include "BeginState.hpp"
#include "ControlFSM.hpp"

//Begin state only waits for preflight request
void BeginState::handleEvent(ControlFSM& fsm, const EventData& event) {
	if(event.request == RequestType::PREFLIGHT) {
		fsm.transitionTo(ControlFSM::PREFLIGHTSTATE, this);
	} else {
		fsm.handleFSMInfo("Invalid transiton");
	}
}

void BeginState::stateBegin(ControlFSM& fsm, const EventData& event) {
	//Nothing to do here really
}

void BeginState::loopState(ControlFSM& fsm) {
	//Begin state only wait for an event - nothing really to do here
}



