#include "control_fsm/LandState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/ControlFSM.hpp"

LandState::LandState() {
	_setpoint.type_mask = default_mask | SETPOINT_TYPE_LAND;
	_setpoint.position.z = -1; //Shouldnt matter
}

void LandState::handleEvent(ControlFSM& fsm, const EventData& event) {
	if(event.isValidRequest()) {
		if(event.request == RequestType::ABORT) {
			handleAbort(fsm);
			return;
		} else {
			fsm.handleFSMWarn("Illegal transition request!");
		}
	} else if(event.eventType == EventType::GROUNDDETECTED) {
		//Land is finished
		if(_cmd.isValidCMD()) {
			//Only landxy should occur!
			if(_cmd.commandType == CommandType::LANDXY) {
				_cmd.finishCMD();
			}
			_cmd = EventData();
		}
		fsm.transitionTo(ControlFSM::IDLESTATE, this, event);
	} else if(event.isValidCMD()) {
		handleCMD(fsm, event);
	}
}

void LandState::stateBegin(ControlFSM& fsm, const EventData& event) {
	if(event.isValidCMD()) {
		_cmd = event;
		_cmd.sendFeedback("Landing!");
	}
	const geometry_msgs::PoseStamped* pPose = fsm.getPositionXYZ();
	if(pPose != nullptr) {
		_setpoint.position.x = pPose->pose.position.x;
		_setpoint.position.y = pPose->pose.position.y;
		//Set yaw setpoint based on current rotation
		//TODO Make sure this cast works fine
		_setpoint.yaw = (float) fsm.getMavrosCorrectedYaw();
	} else {
		//Should never occur
		RequestEvent abortEvent(RequestType::ABORT);
		if(_cmd.isValidCMD()) {
			_cmd.eventError("No valid position");
			_cmd = EventData();
		}
		fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, abortEvent);
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

void LandState::handleAbort(ControlFSM &fsm) {
	if(_cmd.isValidCMD()) {
		_cmd.eventError("ABORT request sent. Aborting command");
		_cmd = EventData();
	}
	RequestEvent event(RequestType::ABORT);
	fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event);
}

void LandState::handleCMD(ControlFSM &fsm, const EventData &event) {
	if(event.isValidCMD()) {
		if(_cmd.isValidCMD()) {
			fsm.handleFSMWarn("ABORT should be sent before new command!");
			event.eventError("CMD rejected!");
		} else {
			fsm.handleFSMWarn("Not accepting CMDs before land is completed!");
			event.eventError("CMD rejected!");
		}
	} else {
		fsm.handleFSMError("Invalid CMD!");
	}
}
