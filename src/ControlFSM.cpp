#include "control_fsm/ControlFSM.hpp"
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#ifndef PI_HALF
#define PI_HALF 1.57079632679
#endif

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
ManualFlightState ControlFSM::MANUALFLIGHTSTATE;

//Change the current running state - be carefull to only change into an allowed state
void ControlFSM::transitionTo(StateInterface& state, StateInterface* pCaller, const EventData& event) {
	//Only current running state is allowed to change state
	if(getState() == pCaller) {
		//Run stateEnd on current running state before transitioning
		getState()->stateEnd(*this, event);
		//Set the current state pointer
		_stateVault._pCurrentState = &state;
		handleFSMInfo("Current state: " + getState()->getStateName());
		//Pass event to new current state
		getState()->stateBegin(*this, event);
		//Notify state has changed
		_onStateChanged();
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
	if(event.eventType == EventType::MANUAL) {
		//If drone entered manual mode: Abort current operation, and go to stable.
		EventData abortEvent;
		abortEvent.eventType = EventType::REQUEST;
		abortEvent.request = RequestType::ABORT;
		handleEvent(abortEvent);
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
	_onFSMError(errMsg);
}

//Send info message to user via ROS
void ControlFSM::handleFSMInfo(std::string infoMsg) {
	ROS_INFO("%s",(std::string("[Control FSM] ") + infoMsg).c_str());
	_onFSMInfo(infoMsg);
}

//Send warning to user via ROS
void ControlFSM::handleFSMWarn(std::string warnMsg) {
	ROS_WARN("%s", (std::string("[Control FSM] ") + warnMsg).c_str());
	_onFSMWarn(warnMsg);
}

//Send debug message to user via ROS
void ControlFSM::handleFSMDebug(std::string debugMsg) {
	ROS_DEBUG("%s", (std::string("[Control FSM] ") + debugMsg).c_str());
}

void ControlFSM::setPosition(const geometry_msgs::PoseStamped& pose) {
	//TODO Set _dronePosition.valid to false if position is not valid
	if(!_dronePosition.isSet) {
		_dronePosition.isSet = true;
	}
	_dronePosition.position = pose;
	_dronePosition.validXY = true; //TODO Add checks here
}

const geometry_msgs::PoseStamped* ControlFSM::getPositionXYZ() {
	if(!_dronePosition.isSet) {
		handleFSMError("Position has not been set!!");
		return nullptr;
	}
	return _dronePosition.validXY ? &_dronePosition.position : nullptr;
}

double ControlFSM::getOrientationYaw() {
	double quatX = _dronePosition.position.pose.orientation.x;
	double quatY = _dronePosition.position.pose.orientation.y;
	double quatZ = _dronePosition.position.pose.orientation.z;
	double quatW = _dronePosition.position.pose.orientation.w;
	tf2::Quaternion q(quatX, quatY, quatZ, quatW);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	//Subtracting PI halfs to correct for a bug in mavros (90 degree offset)
	return yaw;
}

double ControlFSM::getMavrosCorrectedYaw() {
	return getOrientationYaw() - PI_HALF;
}

double ControlFSM::getPositionZ() {
	if(!_dronePosition.isSet) {
		handleFSMError("Position has not been set!!");
	}
 	return _dronePosition.position.pose.position.z;
 }






