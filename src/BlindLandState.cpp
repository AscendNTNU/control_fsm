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
		if(_cmd.eventType == EventType::COMMAND) {
			fsm.transitionTo(ControlFSM::IDLESTATE, this, _cmd);
			_cmd = EventData(); 
		} else {
			fsm.transitionTo(ControlFSM::IDLESTATE, this, event);
		}
	} else if(event.eventType == EventType::COMMAND) {
		_cmd = event; //COPY event and use it after finished landing.
	} else {
		fsm.handleFSMDebug("Event ignored!");
	}
}

void BlindLandState::stateBegin(ControlFSM& fsm, const EventData& event) {
	//TODO Is this override really needed? Decide!
}

void BlindLandState::loopState(ControlFSM& fsm) {
	//TODO Is this override really needed?
}

const mavros_msgs::PositionTarget* BlindLandState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint;
}



