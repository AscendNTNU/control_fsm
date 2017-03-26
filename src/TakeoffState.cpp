#include "control_fsm/TakeoffState.hpp" 
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/ControlFSM.hpp"

TakeoffState::TakeoffState() {
	_setpoint = mavros_msgs::PositionTarget();
	_setpoint.type_mask = default_mask | SETPOINT_TYPE_TAKEOFF;
}

void TakeoffState::handleEvent(ControlFSM& fsm, const EventData& event) {
	//TODO Handle events
}

void TakeoffState::stateBegin(ControlFSM& fsm, const EventData& event) {
	//TODO Implement takeoff
}

void TakeoffState::loopState(ControlFSM& fsm) {
	//TODO Implement takeoff
}

const mavros_msgs::PositionTarget& TakeoffState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return _setpoint; //Will generate error
}
