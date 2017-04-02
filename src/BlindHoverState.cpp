#include "control_fsm/BlindHoverState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/ControlFSM.hpp"
#include "control_fsm/EventData.hpp"

BlindHoverState::BlindHoverState() {
	_setpoint.type_mask = default_mask | IGNORE_PX | IGNORE_PY;
}

void BlindHoverState::handleEvent(ControlFSM& fsm, const EventData& event) {
	//TODO Handle incoming events when blind hovering
	if(event.eventType == EventType::COMMAND) {
		_cmd = event; //Hold event until position is regained.
	} else if(event.eventType == EventType::REQUEST) {
		if(event.request == RequestType::BLINDLAND) {
			fsm.transitionTo(ControlFSM::BLINDLANDSTATE, this, event);
		}
	}
}

void BlindHoverState::stateBegin(ControlFSM& fsm, const EventData& event ) {
	//If full position is valid - no need to blind hover
	if(fsm.getPositionXYZ() != nullptr) {
		if(event.eventType == EventType::COMMAND) {
			fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event); //Pass command on to next state
		} else {
			EventData rEvent;
			rEvent.eventType = EventType::REQUEST;
			rEvent.request = RequestType::POSHOLD;
			fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, rEvent);
		}
		return;
	}
	_setpoint.position.z = BLIND_HOVER_ALTITUDE;
}

void BlindHoverState::loopState(ControlFSM& fsm) {
	//Transition to position hold when position is valid.
	if(fsm.getPositionXYZ() != nullptr) {
		if(_cmd.eventType == EventType::COMMAND) {
			fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, _cmd);
			_cmd = EventData(); //Reset _cmd
			return;
		}
		EventData event;
		event.eventType = EventType::REQUEST;
		event.request = RequestType::POSHOLD;
		fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event);
	}
}

const mavros_msgs::PositionTarget* BlindHoverState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint;
}