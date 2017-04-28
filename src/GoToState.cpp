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
				_cmd = EventData();
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
	//Has not arrived yet
	_delayTransition.enabled = false;
	//TODO Implement rest of stateBegin
	if(!event.positionGoal.valid) {
		if(_cmd.isValidCMD()) {
			event.eventError("No valid position target");
			_cmd = EventData();
		}
		RequestEvent nEvent(RequestType::ABORT);
		fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, nEvent);
		return;
	}
	//Set up nodehandle and load all paramaters first time
	if(_pnh == nullptr) {
		stateInit(fsm);
	}

	//Sets setpoint to current position - until planner is done
	const geometry_msgs::PoseStamped* pose = fsm.getPositionXYZ();
	_setpoint.position.x = pose->pose.position.x;
	_setpoint.position.y = pose->pose.position.y;

	//Z setpoint can be set right away
	_setpoint.position.z = event.positionGoal.z;
	//Set yaw setpoint to current orientation
	_setpoint.yaw = fsm.getOrientationYaw();

	bool xWithinReach = (std::fabs(pose->pose.position.x - event.positionGoal.x) < _destReachedMargin);
	bool yWithinReach = (std::fabs(pose->pose.position.y - event.positionGoal.y) < _destReachedMargin);

	//If only altitude is different, no need for pathplanner
	if(xWithinReach && yWithinReach) {
        if(_cmd.isValidCMD()) {
            _cmd.sendFeedback("Already at correct X and Y - no need for pathplan");
        }
		return;
	}
    if(_cmd.isValidCMD()) {
        _cmd.sendFeedback("Planning path to target!");
    }
	//Send desired goal to path planner
	geometry_msgs::Point32 destPoint;
	geometry_msgs::Point32 currentPos;
	//Only x and y is used
	destPoint.x = event.positionGoal.x;
	destPoint.y = event.positionGoal.y;
	currentPos.x = pose->pose.position.x;
	currentPos.y = pose->pose.position.y;
	/*
	ros need a little delay after advertising for all connection to be made
	This is a bit hacky solution to publish the target and position as soon as the 
	publisher are ready without blocking the FSM.
	*/

	_safePublisher.completed = false;
	_safePublisher.publish = [destPoint, currentPos, this]() {
		//Only subscribes if 
		if(_targetPub.getNumSubscribers() == 0 || _posPub.getNumSubscribers() == 0) {
			return;
		}
		_targetPub.publish(destPoint);
		_posPub.publish(currentPos);
		_safePublisher.completed = true;

	};
	fsm.handleFSMInfo("Sent target and position to planner, waiting for result!");
	

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
    		_cmd = EventData();
    	}
    	fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event);
    	return;
    }

    //TODO Add check for yaw
	bool xWithinReach = (std::fabs(pPose->pose.position.x - _cmd.positionGoal.x) <= _destReachedMargin);
	bool yWithinReach = (std::fabs(pPose->pose.position.y - _cmd.positionGoal.y) <= _destReachedMargin);
	bool zWithinReach = (std::fabs(pPose->pose.position.z - _cmd.positionGoal.z) <= _destReachedMargin);

	//If destination is reached, begin transition to another state
	if(xWithinReach && yWithinReach && zWithinReach) {
		//Hold current position for a duration - avoiding unwanted velocity before doing anything else
		if(!_delayTransition.enabled) {
			_delayTransition.started = ros::Time::now();
			_delayTransition.enabled = true;
		}
		if(ros::Time::now() - _delayTransition.started < _delayTransition.delayTime) {
			return;
		} 
		if(_cmd.isValidCMD()) {
			switch(_cmd.commandType) {
				case CommandType::LANDXY:
					fsm.transitionTo(ControlFSM::LANDSTATE, this, _cmd);
					break;
				case CommandType::LANDGB:
					fsm.transitionTo(ControlFSM::TRACKGBSTATE, this, _cmd);
					break;
				case CommandType::GOTOXYZ:
					_cmd.finishCMD();
					RequestEvent doneEvent(RequestType::POSHOLD);
					fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, doneEvent);
					break;
			}
		} else {
			RequestEvent posHoldEvent(RequestType::POSHOLD);
			fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, posHoldEvent);
		}
		_delayTransition.enabled = false;
		//Destination reached, no need to excecute the rest of the function
		return;
	} else {
		_delayTransition.enabled = false;
	}

	//Make sure points are published!!
	if(!_safePublisher.completed) {
		_safePublisher.publish();
	}
	
	//Only continue if there is a valid plan available
	if(!_currentPlan.valid) {
		return;
	} else if(!(bool)_currentPlan.plan.feasibility || _currentPlan.plan.arrayOfPoints.size() <= 0) {
		//A plan is recieved, but there are no points. 
		fsm.handleFSMError("Recieved empty path plan");
		RequestEvent abortEvent(RequestType::ABORT);
		if(_cmd.isValidCMD()) {
			_cmd.eventError("No feasable path to target");
		}
		fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, abortEvent);
		return;
	}

	//Get current setpoint from plan
	auto currentPoint = _currentPlan.plan.arrayOfPoints[_currentPlan.index];
	//Check if we are close enough to current setpoint
	xWithinReach = (std::fabs(pPose->pose.position.x - currentPoint.x) <= _setpointReachedMargin);
	yWithinReach = (std::fabs(pPose->pose.position.y - currentPoint.y) <= _setpointReachedMargin);
	if(xWithinReach && yWithinReach) {
		//If there are a new setpoint in the plan, change to it.
		if(_currentPlan.plan.arrayOfPoints.size() > (_currentPlan.index + 1)) {
			++_currentPlan.index;
			currentPoint = _currentPlan.plan.arrayOfPoints[_currentPlan.index];
		}
	}
	//Set setpoint x and y
	_setpoint.position.x = currentPoint.x;
	_setpoint.position.y = currentPoint.y;
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
    _cmd.sendFeedback("New path plan recieved!");
	_currentPlan.plan = *msg;
	_currentPlan.valid = true;
	_currentPlan.index = 0;
}

void GoToState::stateInit(ControlFSM& fsm) {
	_pnh.reset(new ros::NodeHandle());	
	ros::NodeHandle n("~");
	if(!n.getParam("control_planner_plan", _planSubTopic)) {
		fsm.handleFSMWarn("No planner plan topic found, using default: " + _planSubTopic);
	}
	if(!n.getParam("control_planner_position", _posPubTopic)) {
		fsm.handleFSMWarn("No planner position topic found, using default: " + _posPubTopic);
	}
	if(!n.getParam("control_planner_target", _targetPubTopic)) {
		fsm.handleFSMWarn("No planner target topic found, using default: " + _targetPubTopic);
	}

	float tempGoToHoldDestTime = -10;
	if(n.getParam("goto_hold_dest_time", tempGoToHoldDestTime) && tempGoToHoldDestTime > 0) {
		fsm.handleFSMInfo("GoTo destionation hold time param found: " + std::to_string(tempGoToHoldDestTime));
		_delayTransition.delayTime = ros::Duration(tempGoToHoldDestTime);
	} else {
		fsm.handleFSMWarn("No valid param goto_hold_dest_time found, using default: " + std::to_string(DEFAULT_DEST_REACHED_DELAY));
		_delayTransition.delayTime = ros::Duration(DEFAULT_DEST_REACHED_DELAY);
	}

	float tempDestReachedMargin = -10;
	if(n.getParam("dest_reached_margin", tempDestReachedMargin) && tempDestReachedMargin > 0) {
		fsm.handleFSMInfo("Destination reached param found: " + std::to_string(tempDestReachedMargin));
		_destReachedMargin = tempDestReachedMargin;
	} else {
		fsm.handleFSMWarn("No valid param dest_reached_margin found, using default: " + std::to_string(DEFAULT_DEST_REACHED_MARGIN));
		_destReachedMargin = DEFAULT_DEST_REACHED_MARGIN;
	}	
	float tempSetpReachedMargin = -10;
	if(n.getParam("setp_reached_margin", tempSetpReachedMargin)) {
		if(std::fabs(_setpointReachedMargin - tempSetpReachedMargin) > 0.001 && tempSetpReachedMargin > 0) {
			fsm.handleFSMInfo("Setpoint reached param found: " + std::to_string(tempSetpReachedMargin));
			_setpointReachedMargin = tempSetpReachedMargin;
		}
	} else {
		fsm.handleFSMWarn("No param setp_reached_margin found, using default: " + std::to_string(DEFAULT_SETPOINT_REACHED_MARGIN));
		_setpointReachedMargin = DEFAULT_SETPOINT_REACHED_MARGIN;
	}
	
	_posPub = _pnh->advertise<geometry_msgs::Point32>(_posPubTopic, 1);
	_targetPub = _pnh->advertise<geometry_msgs::Point32>(_targetPubTopic , 1);
	_planSub = _pnh->subscribe(_planSubTopic, 1, &GoToState::pathRecievedCB, this);
	
}





