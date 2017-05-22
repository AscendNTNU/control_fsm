#include "control_fsm/EstimateAdjustState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/ControlFSM.hpp"

EstimateAdjustState::EstimateAdjustState() {
	_setpoint.type_mask = default_mask | IGNORE_PX | IGNORE_PY; //State should transition before this is needed, but just in case
}

void EstimateAdjustState::handleEvent(ControlFSM& fsm, const EventData& event) {
	if(event.isValidCMD()) {
		handleCMD(fsm, event);
	} else if(event.eventType == EventType::REQUEST) {
		if(event.request == RequestType::ABORT) {
			handleAbort(fsm);
		} else {
			fsm.handleFSMWarn("Illegal transition request");
		}
	} else {
		fsm.handleFSMDebug("Ignoring event");
	}
}

void EstimateAdjustState::loopState(ControlFSM& fsm) {
	//TODO Transition to blindhover as soon as position is invalid
	bool posInvalid = true;

	if(posInvalid) {
		if(_cmd.isValidCMD()) {
			fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, _cmd);
			_cmd = EventData();
		} else {
			RequestEvent event(RequestType::BLINDHOVER);
			fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, event);
		}
	}
}

void EstimateAdjustState::stateBegin(ControlFSM& fsm, const EventData& event) {
	_setpoint.yaw = (float) fsm.getMavrosCorrectedYaw();

}

const mavros_msgs::PositionTarget* EstimateAdjustState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint;
}

void EstimateAdjustState::handleAbort(ControlFSM &fsm) {
	if(_cmd.isValidCMD()) {
		_cmd = EventData();
		_cmd.eventError("ABORT request!");
		fsm.handleFSMDebug("ABORTING command, but estimateadjust cant be aborted!");
	}
}

void EstimateAdjustState::handleCMD(ControlFSM &fsm, const EventData &event) {
	if(event.isValidCMD()) {
		if(_cmd.isValidCMD()) {
			event.eventError("CMD rejected!");
			fsm.handleFSMWarn("ABORT old CMD before sending new!");
		} else {
			_cmd = event;
		}
	} else {
		fsm.handleFSMError("Invalid CMD!");
	}
}

