#include "control_fsm/BlindHoverState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/ControlFSM.hpp"
#include "control_fsm/EventData.hpp"

#define BLIND_HOVER_ALTITUDE 1.0f	

/*
Only blind states (blind hover and blind land) will accept running without valid position.
*/

BlindHoverState::BlindHoverState() {
	_setpoint.type_mask = default_mask | IGNORE_PX | IGNORE_PY;
}

void BlindHoverState::handleEvent(ControlFSM& fsm, const EventData& event) {
	//TODO Handle incoming events when blind hovering
	if(event.isValidCMD()) {
		if(!_cmd.isValidCMD()) {
			_cmd = event; //Hold event until position is regained.
		} else {
			event.eventError("CMD rejected!");
			fsm.handleFSMWarn("ABORT old command first");
		}
	} else if(event.isValidRequest()) {
		if(event.request == RequestType::BLINDLAND) {
			if(_cmd.isValidCMD()) {
				_cmd.eventError("Manual request overriding cmd");
				_cmd = EventData();
			}
			fsm.transitionTo(ControlFSM::BLINDLANDSTATE, this, event);
		} else {
			fsm.handleFSMWarn("Invalid transition request");
		}
	} else {
		fsm.handleFSMInfo("Event ignored!");
	}
}

void BlindHoverState::stateBegin(ControlFSM& fsm, const EventData& event ) {
	//If full position is valid - no need to blind hover
	if(fsm.getPositionXYZ() != nullptr) {
		if(event.eventType == EventType::COMMAND) {
			fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event); //Pass command on to next state
		} else {
			RequestEvent rEvent(RequestType::POSHOLD);
			fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, rEvent);
		}
		return;
	}
	if(event.isValidCMD()) {
		_cmd = event;
	}
	_setpoint.position.z = BLIND_HOVER_ALTITUDE; //TODO Check this design decision (default hover height)
}

void BlindHoverState::loopState(ControlFSM& fsm) {
	//Transition to position hold when position is valid.
	if(fsm.getPositionXYZ() != nullptr) {
		if(_cmd.isValidCMD()) {
			fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, _cmd);
			_cmd = EventData(); //Reset _cmd
		} else {
			RequestEvent event(RequestType::POSHOLD);
			fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event);
		}
	}
}

const mavros_msgs::PositionTarget* BlindHoverState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint;
}