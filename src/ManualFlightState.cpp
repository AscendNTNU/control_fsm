#include "control_fsm/ManualFlightState.hpp"
#include "control_fsm/ControlFSM.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <geometry_msgs/PoseStamped.h>

ManualFlightState::ManualFlightState() {
	_setpoint.type_mask = default_mask; 
}
//Only check for an abort event
void ManualFlightState::handleEvent(ControlFSM& fsm, const EventData& event) {
	if(event.isValidCMD()) {
		event.eventError("CMD rejected!");
		fsm.handleFSMWarn("Drone is not yet active - commands ignored");
	} else if(event.isValidRequest()) {
		if(event.request == RequestType::PREFLIGHT) {
			fsm.handleFSMWarn("Going back to preflight, land drone before offboard");
			fsm.transitionTo(ControlFSM::PREFLIGHTSTATE, this, event);
		} else {
			fsm.handleFSMWarn("Invalid transition request");
		}
	} else if(event.eventType == EventType::AUTONOMOUS) {
		if(fsm._landDetector.isOnGround()) {
			fsm.transitionTo(ControlFSM::IDLESTATE, this, event); //Transition to IDLESTATE
		} else {
			fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, event); //Transition to BLINDHOVERSTATE
		}
	} else {
		fsm.handleFSMInfo("Event ignored");
	}
}

void ManualFlightState::loopState(ControlFSM& fsm) {
	const geometry_msgs::PoseStamped* pPose = fsm.getPositionXYZ();
	if(pPose == nullptr) {
		//Should never occur
		fsm.handleFSMError("Position not valid!!");
		_setpoint.type_mask = default_mask | IGNORE_PX | IGNORE_PY | IGNORE_PZ | IGNORE_YAW;
	} else if(fsm._landDetector.isOnGround()) {
		_setpoint.type_mask = default_mask | SETPOINT_TYPE_IDLE; //Send IDLE setpoints while drone is on ground
	} else {
		_setpoint.type_mask = default_mask;
		_setpoint.position.x = pPose->pose.position.x;
		_setpoint.position.y = pPose->pose.position.y;
		_setpoint.position.z = pPose->pose.position.z;
		_setpoint.yaw = fsm.getMavrosCorrectedYaw();
	}
}


//Returns setpoint
const mavros_msgs::PositionTarget* ManualFlightState::getSetpoint() { 
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint; 
}

void ManualFlightState::handleManual(ControlFSM &fsm) {
	//Already in manual, nothing to do
}
