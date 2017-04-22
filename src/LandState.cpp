#include "control_fsm/LandState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/ControlFSM.hpp"

LandState::LandState() {
	_setpoint.type_mask = default_mask | SETPOINT_TYPE_LAND;
	_setpoint.position.z = -1; //Shouldnt matter
}

void LandState::handleEvent(ControlFSM& fsm, const EventData& event) {
	//TODO Should land ever need to handle commands? ABORT request should be sent before new command 
	if(event.isValidRequest()) {
		if(event.request == RequestType::ABORT) {
			if(_cmd.isValidCMD()) {
				_cmd.eventError("ABORT request sent. Aborting command");
				_cmd = EventData();
			}
			fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event);
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
		if(_cmd.isValidCMD()) {
			fsm.handleFSMWarn("ABORT should be sent before new command!");
			event.eventError("ABORT should be sent before new command!");
		} else {
			fsm.handleFSMWarn("Not accepting CMDs before land is completed!");
		}
	}
}

void LandState::stateBegin(ControlFSM& fsm, const EventData& event) {
	if(event.isValidCMD()) {
		_cmd = event;
	}
	const geometry_msgs::PoseStamped* pPose = fsm.getPositionXYZ();
	if(pPose != nullptr) {
		_setpoint.position.x = pPose->pose.position.x;
		_setpoint.position.y = pPose->pose.position.y;
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