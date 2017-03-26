#include "control_fsm/BlindHoverState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/ControlFSM.hpp"
#include "control_fsm/EventData.hpp"


BlindHoverState::BlindHoverState() {
	_setpoint.type_mask = default_mask | IGNORE_PX | IGNORE_PY;
}

void BlindHoverState::handleEvent(ControlFSM& fsm, const EventData& event) {
	//TODO Implement blind hover events
}

void BlindHoverState::stateBegin(ControlFSM& fsm, const EventData& event ) {
	//TODO Implement blind hover entrypoint (set correct altitude)
}

void BlindHoverState::loopState(ControlFSM& fsm) {
	//TODO Implement blind hover loop (trigger internal transition when position is regained)
}

const mavros_msgs::PositionTarget& BlindHoverState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return _setpoint;
}