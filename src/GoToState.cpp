#include "control_fsm/GoToState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/ControlFSM.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <planner/point.h>

#define DESTINATION_REACHED_THRESHOLD 0.1
#define DEBUG

GoToState::GoToState() {
	_setpoint.type_mask = default_mask;
    //TODO Change topic
	posPub_ = _nh.advertise<planner::point>("/control_path_planner", 10);
}

void GoToState::handleEvent(ControlFSM& fsm, const EventData& event) {
	if(event.isValidRequest()) {
		if(event.request == RequestType::ABORT || event.request == RequestType::POSHOLD) {
			fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event);
		} else {
			fsm.handleFSMWarn("Illegal transiton request");
		}
	} else if(event.isValidCMD()) {
		if(_cmd.isValidCMD()) {
			event.eventError("ABORT request should be sent before new command");
		} else {
			fsm.transitionTo(ControlFSM::GOTOSTATE, this, event); //Transition to itself
		}
	}
}

void GoToState::stateBegin(ControlFSM& fsm, const EventData& event) {
	_cmd = event;
	//TODO Implement rest of stateBegin
	if(!event.positionGoal.valid) {
		if(_cmd.isValidCMD()) {
			event.eventError("No valid position target");
		}
		RequestEvent nEvent(RequestType::ABORT);
		fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, nEvent);
	}
	//Sets setpoint to current position - until planner is done
	const geometry_msgs::PoseStamped* pose = fsm.getPositionXYZ();
	_setpoint.position.x = pose->pose.position.x;
	_setpoint.position.y = pose->pose.position.y;
	_setpoint.position.z = pose->pose.position.z;
	//TODO Set yaw

	planner::point plannerPosition;
	plannerPosition.x = pose->pose.position.x;
	plannerPosition.y = pose->pose.position.y;
}

void GoToState::loopState(ControlFSM& fsm) {
	//TODO Implement GoTo state loop.
    
	//TODO REPLACE THIS - ONLY FOR TESTING PURPOSES
	#ifdef DEBUG
    const geometry_msgs::PoseStamped* pPose = fsm.getPositionXYZ();
    if(pPose == nullptr) {
    	EventData event;
    	event.eventType = EventType::REQUEST;
    	event.request = RequestType::ABORT;
    	if(_cmd.eventType == EventType::COMMAND) {
    		_cmd.eventError("Lost position");
    	}
    }
    if(std::fabs(pPose->pose.position.x - _cmd.positionGoal.x) < DESTINATION_REACHED_THRESHOLD &&
    	std::fabs(pPose->pose.position.y - _cmd.positionGoal.y) < DESTINATION_REACHED_THRESHOLD &&
    	std::fabs(pPose->pose.position.z - _cmd.positionGoal.z) < DESTINATION_REACHED_THRESHOLD) {
    	EventData event;
    	event.eventType = EventType::REQUEST;
    	if(_cmd.isValidCMD()) {
    		switch(_cmd.commandType) {
    			case CommandType::LANDXY:
    				fsm.transitionTo(ControlFSM::LANDSTATE, this, _cmd);
    				break;
    			case CommandType::LANDGB:
    				fsm.transitionTo(ControlFSM::TRACKGBSTATE, this, _cmd);
    				break;
    			default:
    				_cmd.finishCMD();
    				event.request = RequestType::POSHOLD;
    				fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event);
    				break;
    		}
    	} else {
    		event.request = RequestType::POSHOLD;
    		fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event);
    	}
    }
    #endif

}

const mavros_msgs::PositionTarget* GoToState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint;
}



