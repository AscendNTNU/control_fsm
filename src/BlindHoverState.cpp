#include "control_fsm/BlindHoverState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/ControlFSM.hpp"
#include "control_fsm/EventData.hpp"

#define DEFAULT_BLIND_HOVER_ALTITUDE 1.0f	

/*
Only blind states (blind hover and blind land) will accept running without valid position.
*/

BlindHoverState::BlindHoverState() {
	_setpoint.type_mask = default_mask | IGNORE_PX | IGNORE_PY;
	_setpoint.position.z = DEFAULT_BLIND_HOVER_ALTITUDE;
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
		} else if(event.request == RequestType::ABORT){
			if(_cmd.isValidCMD()) {
				fsm.handleFSMInfo("Aborting CMD");
				_cmd.eventError("ABORT");
				_cmd = EventData();
			} else {
				fsm.handleFSMWarn("Can't abort blind hover");
			}
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

	//Set relevant parameters
	//Takeoff altitude
	ros::NodeHandle _nh("~");
	float temp_blind_hover_alt = -10;
	if(_nh.getParam("blind_hover_altitude", temp_blind_hover_alt)) {
		if(std::fabs(_setpoint.position.z - temp_blind_hover_alt) > 0.01 && temp_blind_hover_alt > 0) {
			fsm.handleFSMInfo("Takeoff altitude param found: " + std::to_string(temp_blind_hover_alt));
			_setpoint.position.z = temp_blind_hover_alt;
		}
	} else {
		fsm.handleFSMWarn("No takeoff altitude param found, using default altitude: " + std::to_string(DEFAULT_TAKEOFF_ALTITUDE));
		_setpoint.position.z = DEFAULT_BLIND_HOVER_ALTITUDE;
	}
	_setpoint.yaw = fsm.getOrientationYaw();
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