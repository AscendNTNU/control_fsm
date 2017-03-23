#include "ControlFSM.hpp"

//TODO: Initiate static instances of the different state classes here!!
BeginState ControlFSM::BEGINSTATE;
PreFlightState ControlFSM::PREFLIGHTSTATE;


//Change the current running state - be carefull to only change into an allowed state
void ControlFSM::transitionTo(StateInterface& state, StateInterface* pCaller) {
	//Only current running state is allowed to change state
	if(getState() == pCaller) {
		_stateVault._pCurrentState = &state;
	}
}

//Send external event to current state and to "next" state
void ControlFSM::handleEvent(const EventData& event) {
	if(getState() == nullptr) {
		handleFSMError("Bad implementation of FSM - FSM allways need a state");
		return;
	}
	//Pass event to current running state
	getState()->handleEvent(*this, event);
	//Pass event to new current running state
	getState()->stateBegin(*this, event);
}

//Runs state specific code on current state
void ControlFSM::loopCurrentState(void) {
	if(getState() == nullptr) {
		handleFSMError("Bad implementation of FSM - no current state!!");
		return;
	}
	getState()->loopState(*this);
}


//Change the current running state - be carefull to only change into an allowed state
//Send error message to user
void ControlFSM::handleFSMError(std::string errMsg) {
	std::cout << "[FSM ERROR] " << errMsg << std::endl;
}

//Send info message to user
void ControlFSM::handleFSMInfo(std::string infoMsg) {
	std::cout << "[FSM INFO] " << infoMsg << std::endl;
}


