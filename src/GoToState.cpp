#include "control_fsm/GoToState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/ControlFSM.hpp"

GoToState::GoToState() {
	_setpoint.type_mask = default_mask;
}

void GoToState::handleEvent(ControlFSM& fsm, const EventData& event) {
	if(event.eventType == EventType::REQUEST) {
		if(event.request == RequestType::ABORT || event.request == RequestType::POSHOLD) {
			fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event);
		} else {
			fsm.handleFSMWarn("Illegal transiton request");
		}
	} else if(event.eventType == EventType::COMMAND) {
		if(event.commandType != CommandType::NONE) {
			_cmd = event;
			//TODO Initiate route replan
		} else {
			fsm.handleFSMWarn("Recieved command with commandtype NONE - bug!");
		}
	}
}

void GoToState::stateBegin(ControlFSM& fsm, const EventData& event) {
	_cmd = event; //Copy event data for later decisions
	//TODO Implement rest of stateBegin
}

void GoToState::loopState(ControlFSM& fsm) {
	//TODO Implement GoTo state loop.
    //Remember to offload any kind of heavy computing to another thread/node
}

const mavros_msgs::PositionTarget* GoToState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint;
}



