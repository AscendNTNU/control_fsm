#include "control_fsm/ControlFSM.hpp"
#include <ros/ros.h>

//TODO: Initiate static instances of the different state classes here!!
BeginState ControlFSM::BEGINSTATE;
PreFlightState ControlFSM::PREFLIGHTSTATE;
IdleState ControlFSM::IDLESTATE;
TakeoffState ControlFSM::TAKEOFFSTATE;
BlindHoverState ControlFSM::BLINDHOVERSTATE;
PositionHoldState ControlFSM::POSITIONHOLDSTATE;
ShutdownState ControlFSM::SHUTDOWNSTATE;
EstimateAdjustState ControlFSM::ESTIMATEADJUSTSTATE;
TrackGBState ControlFSM::TRACKGBSTATE;
InteractGBState ControlFSM::INTERACTGBSTATE;
GoToState ControlFSM::GOTOSTATE;
LandState ControlFSM::LANDSTATE;
BlindLandState ControlFSM::BLINDLANDSTATE;

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
		handleFSMError("Transition request made by another state");
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

//Send error message to user via ROS
void ControlFSM::handleFSMError(std::string errMsg) {
	ROS_ERROR("%s", (std::string("[Control FSM] ") + errMsg).c_str());
}

//Send info message to user via ROS
void ControlFSM::handleFSMInfo(std::string infoMsg) {
	ROS_INFO("%s",(std::string("[Control FSM] ") + infoMsg).c_str());
}

//Send warning to user via ROS
void ControlFSM::handleFSMWarn(std::string warnMsg) {
	ROS_WARN("%s", (std::string("[Control FSM] ") + warnMsg).c_str());
}

//Send debug message to user via ROS
void ControlFSM::handleFSMDebug(std::string debugMsg) {
	ROS_DEBUG("%s", (std::string("[Control FSM] ") + debugMsg).c_str());
}

void ControlFSM::setPosition(const geometry_msgs::PoseStamped& pose) {
	//TODO Set _dronePosition.valid to false if position is not valid
	_dronePosition.position = pose;
	_dronePosition.validXY = true; //TODO Add checks here
}




