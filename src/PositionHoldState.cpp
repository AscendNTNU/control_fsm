#include "control_fsm/PositionHoldState.hpp"
#include "control_fsm/setpoint_msg_defines.h"

//Constructor sets default setpoint type mask
PositionHoldState::PositionHoldState() {
	_setpoint.type_mask = default_mask;
}

void PositionHoldState::handleEvent(ControlFSM& fsm, const EventData& event) {
	//TODO Handle all possible transitions from this state
}

void PositionHoldState::stateBegin(ControlFSM& fsm, const EventData& event) {
	//TODO Initalize variables so the drone will hold it's position
}

const mavros_msgs::PositionTarget& PositionHoldState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return _setpoint;
}