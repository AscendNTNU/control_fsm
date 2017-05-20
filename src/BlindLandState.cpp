#include "control_fsm/BlindLandState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/ControlFSM.hpp"

BlindLandState::BlindLandState() {
	_setpoint.type_mask = default_mask | IGNORE_PX | IGNORE_PY | SETPOINT_TYPE_LAND;
}

void BlindLandState::handleEvent(ControlFSM& fsm, const EventData& event) {
	if(event.eventType == EventType::GROUNDDETECTED) {
		//Land completed
		fsm.transitionTo(ControlFSM::IDLESTATE, this, event);
	} else if(event.isValidCMD()) {
		//Blind land not part of normal operation. 
		//Any command will be ignored!
		event.eventError("CMD rejected!");
		fsm.handleFSMWarn("Blind land not part of normal operation! Ignoring commands!");
	} else if(event.isValidRequest()) {
		if(event.request == RequestType::ABORT) {
			fsm.handleFSMWarn("Can't ABORT blind land!");
		} else {
			fsm.handleFSMWarn("Illegal transition request!");
		}
	} else {
		fsm.handleFSMDebug("Event ignored!");
	}
}

void BlindLandState::stateBegin(ControlFSM& fsm, const EventData& event) {
	_setpoint.yaw = (float) fsm.getMavrosCorrectedYaw();
}

void BlindLandState::loopState(ControlFSM& fsm) {
	//TODO Is this override really needed?
}

const mavros_msgs::PositionTarget* BlindLandState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint;
}



