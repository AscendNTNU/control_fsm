#include "control_fsm/TakeoffState.hpp" 
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/ControlFSM.hpp"

#define TAKEOFF_ALTITUDE 1.0f 
#define TAKEOFF_ALTITUDE_REACHED_THRESHOLD 0.1

TakeoffState::TakeoffState() {
	_setpoint = mavros_msgs::PositionTarget();
	_setpoint.type_mask = default_mask | SETPOINT_TYPE_TAKEOFF;
}

void TakeoffState::handleEvent(ControlFSM& fsm, const EventData& event) {
	//TODO Handle events
	if(event.eventType == EventType::COMMAND) {
		_cmd = event;
	} else if(event.eventType == EventType::REQUEST) {
		if(event.request == RequestType::ABORT && _cmd.eventType == EventType::COMMAND) {
			_cmd = EventData(); //Aborting commands, but will still continue takeoff
			fsm.handleFSMInfo("Command aborted, but takeoff can't be aborted");
		} else {
			fsm.handleFSMWarn("Illegal transition request");
		}
	} else {
		fsm.handleFSMInfo("Event ignored");
	}
}

void TakeoffState::stateBegin(ControlFSM& fsm, const EventData& event) {
	if(event.eventType == EventType::COMMAND) {
		_cmd = event;
	}
}

void TakeoffState::loopState(ControlFSM& fsm) {
	double z = fsm.getPositionZ();
	if(z >= TAKEOFF_ALTITUDE - TAKEOFF_ALTITUDE_REACHED_THRESHOLD) {
		if(_cmd.eventType == EventType::COMMAND) {
			fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, _cmd);
			_cmd = EventData();
		} else {
			EventData event;
			event.eventType == EventType::REQUEST;
			event.request == RequestType::BLINDHOVER;
			fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, event);
		}
	}
}

const mavros_msgs::PositionTarget* TakeoffState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint; //Will generate error
}
