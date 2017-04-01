#include "control_fsm/LandState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>

LandState::LandState() {
	_setpoint.type_mask = default_mask | SETPOINT_TYPE_LAND;
}

void LandState::handleEvent(ControlFSM& fsm, const EventData& event) {
	//TODO Handle transitions
}

void LandState::stateBegin(ControlFSM& fsm, const EventData& event) {
	//TODO Complete LandXY command
}

void LandState::loopState(ControlFSM& fsm) {
	//Autotransition to IDLE when completed
	//Finish current commands
}

const mavros_msgs::PositionTarget* LandState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint;
}