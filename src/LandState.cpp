#include "control_fsm/LandState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/ControlFSM.hpp"

LandState::LandState() {
	_setpoint.type_mask = default_mask | SETPOINT_TYPE_LAND;
}

void LandState::handleEvent(ControlFSM& fsm, const EventData& event) {
	//TODO Should land ever need to handle commands? ABORT request should be sent before new command 
	if(event.eventType == EventType::REQUEST) {
		if(event.request == RequestType::ABORT) {
			fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event);
			return;
		}
	} else if(event.eventType == EventType::GROUNDDETECTED) {
		//Land is finished
		if(_cmd.eventType == EventType::COMMAND) {
			_cmd.finishCMD();
			_cmd = EventData();
		}
		fsm.transitionTo(ControlFSM::IDLESTATE, this, event);
	} else if(event.eventType == EventType::COMMAND) {
		fsm.handleFSMError("ABORT should be sent before new command!");
		event.eventError("ABORT should be sent before new command!");
	}
}

void LandState::stateBegin(ControlFSM& fsm, const EventData& event) {
	if(event.eventType == EventType::COMMAND) {
		_cmd = event;
	}
}

void LandState::loopState(ControlFSM& fsm) {
	//Autotransition to IDLE when completed
	//Finish current commands
}

const mavros_msgs::PositionTarget* LandState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint;
}