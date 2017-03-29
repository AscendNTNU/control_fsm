#include "control_fsm/GoToState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>

GoToState::GoToState() {
	_setpoint.type_mask = default_mask;
}

void GoToState::handleEvent(ControlFSM& fsm, const EventData& event) {
	//TODO Implement handle event
}

void GoToState::stateBegin(ControlFSM& fsm, const EventData& event) {
	_lastEvent = event; //Copy event data for later decisions
	//TODO Implement rest of stateBegin
}

void GoToState::loopState(ControlFSM& fsm) {
	//TODO Implement GoTo state loop.
}

const mavros_msgs::PositionTarget& GoToState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return _setpoint;
}



