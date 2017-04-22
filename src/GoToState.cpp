#include "control_fsm/GoToState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/ControlFSM.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <geometry_msgs/Point32.h>
#include <ascend_msgs/PathPlannerPlan.h>

#define DESTINATION_REACHED_THRESHOLD 0.1
#define DEBUG

GoToState::GoToState() {
	_setpoint.type_mask = default_mask;
}

void GoToState::handleEvent(ControlFSM& fsm, const EventData& event) {
	if(event.isValidRequest()) {
		if(event.request == RequestType::ABORT) {
			if(_cmd.isValidCMD()) {
				_cmd.eventError("ABORT");
			}
			fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event);
		} else if(event.request == RequestType::POSHOLD) {
			if(_cmd.isValidCMD()) {
				fsm.handleFSMWarn("ABORT CMD before sending manual request!");
			} else {
				fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event);
			}
		} else if(event.request == RequestType::GOTO) {
			if(_cmd.isValidCMD()) {
				fsm.handleFSMWarn("ABORT CMD before sending manual request!");
			} else {
				fsm.transitionTo(ControlFSM::GOTOSTATE, this, event);
			}
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
	_isActive = true;
	_cmd = event;
	//Current plan is invalid until new plan is recieved
	_currentPlan.valid = false;
	//TODO Implement rest of stateBegin
	if(!event.positionGoal.valid) {
		if(_cmd.isValidCMD()) {
			event.eventError("No valid position target");
		}
		RequestEvent nEvent(RequestType::ABORT);
		fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, nEvent);
		return;
	}

	if(_pnh == nullptr) {
		stateInit(fsm);
	}

	float tempDestReachedMargin = -10;
	if(_pnh->getParam("dest_reached_margin", tempDestReachedMargin)) {
		if(std::fabs(_destReachedMargin - tempDestReachedMargin) > 0.001 && tempDestReachedMargin > 0) {
			fsm.handleFSMInfo("Destination reached param found: " + std::to_string(tempDestReachedMargin));
			_destReachedMargin = tempDestReachedMargin;
		}
	} else {
		fsm.handleFSMWarn("No param dest_reached_margin found, using default: " + std::to_string(DEFAULT_DEST_REACHED_MARGIN));
		_destReachedMargin = DEFAULT_DEST_REACHED_MARGIN;
	}


	//Sets setpoint to current position - until planner is done
	const geometry_msgs::PoseStamped* pose = fsm.getPositionXYZ();
	_setpoint.position.x = pose->pose.position.x;
	_setpoint.position.y = pose->pose.position.y;
	_setpoint.position.z = pose->pose.position.z;
	//TODO Set yaw

	bool xWithinReach = (std::fabs(pose->pose.position.x - event.positionGoal.x) < _destReachedMargin);
	bool yWithinReach = (std::fabs(pose->pose.position.y - event.positionGoal.y) < _destReachedMargin);

	//If only altitude is different, no need for pathplanner
	if(xWithinReach && yWithinReach) {
		_setpoint.position.z = event.positionGoal.z;

	} else {
		geometry_msgs::Point32 destPoint;
		//Only x and y is used
		destPoint.x = event.positionGoal.x;
		destPoint.y = event.positionGoal.y;
		//Send desired target to pathplanner
		_targetPub.publish(destPoint);
	}

}

void GoToState::stateEnd(ControlFSM& fsm, const EventData& event) {
	_isActive = false;
}

void GoToState::loopState(ControlFSM& fsm) {

	//Get position
	const geometry_msgs::PoseStamped* pPose = fsm.getPositionXYZ();
	//Should never occur, but just in case
    if(pPose == nullptr) {
    	EventData event;
    	event.eventType = EventType::POSLOST;
    	if(_cmd.isValidCMD()) {
    		_cmd.eventError("No position");
    	}
    	fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event);
    	return;
    }

	//TODO Implement GoTo state loop.
	bool xWithinReach = (std::fabs(pose->pose.position.x - event.positionGoal.x) < _destReachedMargin);
	bool yWithinReach = (std::fabs(pose->pose.position.y - event.positionGoal.y) < _destReachedMargin);
	bool zWithinReach = (std::fabs(pose->pose.position.z - event.positionGoal.z) < _destReachedMargin);

	//If destination is reached, transition to another state
	if(xWithinReach && yWithinReach && zWithinReach) {
		if(_cmd.isValidCMD()) {
			switch(_cmd.commandType) {
				case CommandType::LANDXY:
					fsm.transitionTo(ControlFSM::LANDSTATE, this, _cmd);
					break;
				case CommandType::LANDGB:
					fsm.transitionTo(ControlFSM::TRACKGBSTATE, this, _cmd);
					break;
				case CommandType::GOTO:
					_cmd.finishCMD();
					RequestEvent doneEvent(RequestType::POSHOLD);
					fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, doneEvent);
					break;
			}
		} else {
			RequestEvent posHoldEvent(RequestType::POSHOLD);
			fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, posHoldEvent);
		}
		//Destination reached, no need to excecute the rest of the function
		return;
	}

}

const mavros_msgs::PositionTarget* GoToState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint;
}

void GoToState::pathRecievedCB(const ascend_msgs::PathPlannerPlan::ConstPtr& msg) {
	//Ignore callback if state is not active
	if(!_isActive) {
		return;
	}
	_currentPlan.plan = *msg;
	_currentPlan.valid = true;
}

void GoToState::stateInit(ControlFSM& fsm) {
		_pnh.reset(new ros::NodeHandle("~"));
		if(!_pnh->getParam("control_planner_plan", _planSubTopic)) {
			fsm.handleFSMWarn("No planner topic found, using default: " + _planSubTopic);
		}
		if(!_pnh->getParam("control_planner_position", _posPubTopic)) {
			fsm.handleFSMWarn("No planner topic found, using default: " + _posPubTopic);
		}
		if(!_pnh->getParam("control_planner_target", _targetPubTopic)) {
			fsm.handleFSMWarn("No planner topic found, using default: " + _targetPubTopic);
		}
		_posPub = _pnh->advertise<geometry_msgs::Point32>(_posPubTopic, 1);
		_targetPub = _pnh->advertise<geometry_msgs::Point32>(_targetPubTopic , 1);
		_planSub = _pnh->subscribe(_planSubTopic, 1, &GoToState::pathRecievedCB, this);

}



