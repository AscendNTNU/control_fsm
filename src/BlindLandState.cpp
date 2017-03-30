#include "control_fsm/BlindLandState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>

BlindLandState::BlindLandState() {
	_setpoint.type_mask = default_mask | IGNORE_PX | IGNORE_PY | SETPOINT_TYPE_LAND;
}

void BlindLandState::handleEvent(ControlFSM& fsm, const EventData& event) {
	//TODO Handle event
}

void BlindLandState::stateBegin(ControlFSM& fsm, const EventData& event) {
	//TODO Is this override really needed? Decide!
}

void BlindLandState::loopState(ControlFSM& fsm) {
	//TODO Transition to IDLE when landed
}

const mavros_msgs::PositionTarget* BlindLandState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint;
}



