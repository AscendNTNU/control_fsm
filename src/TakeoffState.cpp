#include "control_fsm/TakeoffState.hpp" 
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/ControlFSM.hpp"

#define TAKEOFF_ALTITUDE 1.0f 
#define TAKEOFF_ALTITUDE_REACHED_THRESHOLD 0.1

TakeoffState::TakeoffState() {
	_setpoint = mavros_msgs::PositionTarget();
	_setpoint.type_mask = default_mask | SETPOINT_TYPE_TAKEOFF;
	_setpoint.position.z = TAKEOFF_ALTITUDE;
}

void TakeoffState::handleEvent(ControlFSM& fsm, const EventData& event) {
	//TODO Handle events
	if(event.isValidCMD()) {
		_cmd = event;
	} else if(event.isValidRequest()) {
		if(event.request == RequestType::ABORT && _cmd.isValidCMD()) {
			_cmd.eventError("Aborting command");
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
	if(event.isValidCMD()) {
		_cmd = event;
	}
	const geometry_msgs::PoseStamped* pose = fsm.getPositionXYZ();
	//If no position is available - abort takeoff
	if(pose == nullptr) {
		fsm.handleFSMError("No position available");
		if(_cmd.isValidCMD()) {
			_cmd.eventError("No position available");
			_cmd = EventData();
		}
		RequestEvent abortEvent(RequestType::ABORT);
		fsm.transitionTo(ControlFSM::IDLESTATE, this, abortEvent);
		return;
	}
	//Set takeoff setpoint to current XY position
	_setpoint.position.x = pose->pose.position.x;
	_setpoint.position.y = pose->pose.position.y;
}

void TakeoffState::loopState(ControlFSM& fsm) {
	//TODO Reimplement this - only for testing
	double z = fsm.getPositionZ();
	if(z >= TAKEOFF_ALTITUDE - TAKEOFF_ALTITUDE_REACHED_THRESHOLD) {
		if(_cmd.eventType == EventType::COMMAND) {
			fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, _cmd);
			_cmd = EventData();
		} else {
			RequestEvent event(RequestType::BLINDHOVER);
			fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, event);
		}
	}
}

const mavros_msgs::PositionTarget* TakeoffState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint; //Will generate error
}
