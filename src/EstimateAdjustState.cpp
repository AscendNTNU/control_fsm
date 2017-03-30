#include "control_fsm/EstimateAdjustState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>

EstimateAdjustState::EstimateAdjustState() {
	_setpoint.type_mask = default_mask | IGNORE_PX | IGNORE_PY; //State should transition before this is needed, but just in case
}

void EstimateAdjustState::handleEvent(ControlFSM& fsm, const EventData& event) {
	//TODO Handle transitions
}

void EstimateAdjustState::stateBegin(ControlFSM& fsm, const EventData& event) {
	//TODO Go to blind hover
}

const mavros_msgs::PositionTarget* EstimateAdjustState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint;
}

