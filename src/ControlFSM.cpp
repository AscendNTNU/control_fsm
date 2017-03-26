#include "control_fsm/ControlFSM.hpp"
#include <ros/ros.h>

//TODO: Initiate static instances of the different state classes here!!
BeginState ControlFSM::BEGINSTATE;
PreFlightState ControlFSM::PREFLIGHTSTATE;
IdleState ControlFSM::IDLESTATE;
TakeoffState ControlFSM::TAKEOFFSTATE;
BlindHoverState ControlFSM::BLINDHOVERSTATE;
PositionHoldState ControlFSM::POSITIONHOLDSTATE;

//Change the current running state - be carefull to only change into an allowed state
void ControlFSM::transitionTo(StateInterface& state, StateInterface* pCaller, const EventData& event) {
	//Only current running state is allowed to change state
	if(getState() == pCaller) {
		//Set the current state pointer
		_stateVault._pCurrentState = &state;
		handleFSMInfo("Current state: " + getState()->getStateName());
		//Pass event to new current state
		getState()->stateBegin(*this, event);
	} else {
		handleFSMError("Transition request made another state");
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
//Send error message to user via ROS
void ControlFSM::handleFSMError(std::string errMsg) {
	ROS_ERROR("%s", (std::string("[Control FSM] ") + errMsg).c_str());
}

//Send info message to user via ROS
void ControlFSM::handleFSMInfo(std::string infoMsg) {
	ROS_INFO("%s",(std::string("[Control FSM] ") + infoMsg).c_str());
}


