#include "control_fsm/IdleState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/EventData.hpp"
#include "control_fsm/ControlFSM.hpp"

//Sets setpoint type to IDLE
IdleState::IdleState() {
	_setpoint = mavros_msgs::PositionTarget();
	_setpoint.type_mask = default_mask | SETPOINT_TYPE_IDLE;
}

void IdleState::handleEvent(ControlFSM& fsm, const EventData& event) {
	//All commands needs to get to position hold first
	if(event.isValidCMD()) {
		handleCMD(fsm, event);
	} else if(event.isValidRequest()) {
		if(event.request == RequestType::TAKEOFF) {
			fsm.transitionTo(ControlFSM::TAKEOFFSTATE, this, event);
		} else {
			fsm.handleFSMWarn("Invalid transition request");
		}
	} else {
		fsm.handleFSMInfo("Event ignored");
	}
}

const mavros_msgs::PositionTarget* IdleState::getSetpoint() {
	//Sets timestamp, and returns _setpoint as const pointer
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint;
}

void IdleState::handleAbort(ControlFSM &fsm) {
	fsm.handleFSMWarn("Can't abort idle!");
}

void IdleState::handleCMD(ControlFSM &fsm, const EventData &event) {
	if(event.isValidCMD()) {
		fsm.transitionTo(ControlFSM::TAKEOFFSTATE, this, event);
	} else {
		fsm.handleFSMError("Invalid CMD");
	}
}

