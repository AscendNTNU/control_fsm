#include "control_fsm/TakeoffState.hpp" 
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/ControlFSM.hpp"
#include <cmath>
#include <string>

TakeoffState::TakeoffState() {
	_setpoint = mavros_msgs::PositionTarget();
	_setpoint.type_mask = default_mask | SETPOINT_TYPE_TAKEOFF;
	_setpoint.position.z = DEFAULT_TAKEOFF_ALTITUDE;
}

void TakeoffState::handleEvent(ControlFSM& fsm, const EventData& event) {
	//TODO Handle events
	if(event.isValidCMD()) {
		if(_cmd.isValidCMD()) {
			_cmd = event;
		} else {
			event.eventError("Finish old CMD before sending new");
			fsm.handleFSMWarn("ABORT old cmd before sending new");
		}
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
	//Set relevant parameters
	//Takeoff altitude
	ros::NodeHandle _nh("~");
	float temp_takeoff_alt = -10;
	if(_nh.getParam("takeoff_altitude", temp_takeoff_alt)) {
		if(std::fabs(_setpoint.position.z - temp_takeoff_alt) > 0.01 && temp_takeoff_alt > 0) {
			fsm.handleFSMInfo("Takeoff altitude param found: " + std::to_string(temp_takeoff_alt));
			_setpoint.position.z = temp_takeoff_alt;
		}
	} else {
		fsm.handleFSMWarn("No takeoff altitude param found, using default altitude: " + std::to_string(DEFAULT_TAKEOFF_ALTITUDE));
		_setpoint.position.z = DEFAULT_TAKEOFF_ALTITUDE;
	}
	//Takeoff finished threshold
	float temp_alt_margin = -10;
	if(_nh.getParam("altitude_reached_margin", temp_alt_margin)) {
		if(std::fabs(_altitude_reached_margin - temp_alt_margin) > 0.001 && temp_alt_margin > 0) {
			fsm.handleFSMInfo("Takeoff altitude threshold param found: " + std::to_string(temp_alt_margin) );
			_altitude_reached_margin = temp_alt_margin;
		}
	} else {
		fsm.handleFSMWarn("No takeoff altitude param found, using default: " + std::to_string(DEFAULT_TAKEOFF_ALTITUDE_REACHED_MARGIN));
		_altitude_reached_margin = DEFAULT_TAKEOFF_ALTITUDE_REACHED_MARGIN;
	}

	if(event.isValidCMD()) {
		_cmd = event;
		_cmd.sendFeedback("Takeoff!!");
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
	//Set yaw setpoint based on current rotation
	_setpoint.yaw = fsm.getOrientationYaw();;
}

void TakeoffState::loopState(ControlFSM& fsm) {
	double z = fsm.getPositionZ();
	if(z > (_setpoint.position.z - _altitude_reached_margin)) {
		if(_cmd.isValidCMD()) {
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
	return &_setpoint;
}
