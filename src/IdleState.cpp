#include "control_fsm/IdleState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/EventData.hpp"

//Sets setpoint type to IDLE
IdleState::IdleState() {
	_setpoint = mavros_msgs::PositionTarget();
	_setpoint.type_mask = default_mask | SETPOINT_TYPE_IDLE;
}

void IdleState::handleEvent(ControlFSM& fsm, const EventData& event) {
	//TODO Handle events when all states are in place
}

const mavros_msgs::PositionTarget* IdleState::getSetpoint() {
	//Sets timestamp, and returns _setpoint as const reference
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint;
}

