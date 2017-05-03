#include "control_fsm/GoToState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/ControlFSM.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <geometry_msgs/Point32.h>
#include <ascend_msgs/PathPlannerPlan.h>
#include "control_fsm/FSMConfig.hpp"

constexpr double PI = 3.14159265359;
constexpr double PI_HALF = 1.57079632679;


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
	const geometry_msgs::PoseStamped* pPose = fsm.getPositionXYZ();
	_setpoint.position.x = pPose->pose.position.x;
	_setpoint.position.y = pPose->pose.position.y;

	//Z setpoint can be set right away
	_setpoint.position.z = event.positionGoal.z;
	//Set yaw setpoint to desired target yaw
	_setpoint.yaw = fsm.getMavrosCorrectedYaw();

	//Calculate the square distance from drone to target
	double deltaX = pPose->pose.position.x - event.positionGoal.x;
	double deltaY = pPose->pose.position.y - event.positionGoal.y;
	double posXYDistSquare = std::pow(deltaX, 2) + std::pow(deltaY, 2);

	//If only altitude is different, no need for pathplanner
	if(posXYDistSquare <= std::pow(_destReachedMargin, 2)) {
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
	//Only x and y is used
	destPoint.x = event.positionGoal.x;
	destPoint.y = event.positionGoal.y;
	/*
	ros need a little delay after advertising for all connection to be made
	This is a bit hacky solution to publish the target and position as soon as the 
	publisher are ready without blocking the FSM. This guarantees it will be published when
	the planner is listening
	*/
	_safePublisher.completed = false;
	_safePublisher.publish = [destPoint, this]() {
		//Only publish if the path planner is listening
		if(_targetPub.getNumSubscribers() == 0) {
			return;
		}
		_targetPub.publish(destPoint);
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

    double deltaX = pPose->pose.position.x - _cmd.positionGoal.x;
    double deltaY = pPose->pose.position.y - _cmd.positionGoal.y;
    double deltaZ = pPose->pose.position.z - _cmd.positionGoal.z;

	bool xyWithinReach = (std::pow(deltaX, 2) + std::pow(deltaY, 2)) <= std::pow(_destReachedMargin, 2);
	bool zWithinReach = (deltaZ <= FSMConfig::AltitudeReachedMargin);
	bool yawWithinReach = (std::fabs(fsm.getMavrosCorrectedYaw() - _setpoint.yaw) <= _yawReachedMargin);
	//If destination is reached, begin transition to another state
	if(xyWithinReach && zWithinReach && yawWithinReach) {
		//Hold current position for a duration - avoiding unwanted velocity before doing anything else
		if(!_delayTransition.enabled) {
			_delayTransition.started = ros::Time::now();
			_delayTransition.enabled = true;
			if(_cmd.isValidCMD()) {
				_cmd.sendFeedback("Dest reached letting position set before transitioning!");
			}
		}
		//Delay transition
		if(ros::Time::now() - _delayTransition.started < _delayTransition.delayTime) {
			return;
		} 
		//Transition to correct state
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

	/**********************************************************/
	//If the destination is not reached, the loop will continue will run

	//Only run pathplanner if neccesary.
	if(xyWithinReach) {
		_setpoint.position.x = _cmd.positionGoal.x;
		_setpoint.position.y = _cmd.positionGoal.y;
		_setpoint.position.z = _cmd.positionGoal.z;
		return;
	} else {
		//Make sure target point is published!!
		if(!_safePublisher.completed) {
			_safePublisher.publish();
		}
		//Send current position to path planner
    	geometry_msgs::Point32 currentPos;
	    currentPos.x = pPose->pose.position.x;
	    currentPos.y = pPose->pose.position.y;
	    _posPub.publish(currentPos);
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
	double xPos = pPose->pose.position.x;
	double yPos = pPose->pose.position.y;
	//Check if we are close enough to current setpoint
	deltaX = pPose->pose.position.x - currentPoint.x;
	deltaY = pPose->pose.position.y - currentPoint.y;
	if(std::pow(deltaX, 2) + std::pow(deltaY, 2) <= std::pow(_setpointReachedMargin, 2)) {
		//If there are a new setpoint in the plan, change to it.
		if(_currentPlan.plan.arrayOfPoints.size() > (_currentPlan.index + 1)) {
			++_currentPlan.index;
			currentPoint = _currentPlan.plan.arrayOfPoints[_currentPlan.index];
			double dx = currentPoint.x - xPos;
			double dy = currentPoint.y - yPos;
			//Only change yaw if drone needs to travel a larger distance
			if(std::pow(dx, 2) + std::pow(dy, 2) > std::pow(FSMConfig::NoYawCorrectDist, 2)) {
				//-PI_HALF due to mavros bug
				_setpoint.yaw = calculatePathYaw(dx, dy) - PI_HALF;
			}
		}
	}
	//Set setpoint x and y
	_setpoint.position.x = currentPoint.x;
	_setpoint.position.y = currentPoint.y;
}

//Returns valid setpoint
const mavros_msgs::PositionTarget* GoToState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint;
}

//New pathplan is recieved
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

//Initialize state
void GoToState::stateInit(ControlFSM& fsm) {
	//Create new nodehandle
	if(_pnh == nullptr) {
		_pnh.reset(new ros::NodeHandle());	
	}
	//Set state variables
	_delayTransition.delayTime = ros::Duration(FSMConfig::GoToHoldDestTime);
	_destReachedMargin = FSMConfig::DestReachedMargin;
	_setpointReachedMargin = FSMConfig::SetpointReachedMargin;	
	_yawReachedMargin = FSMConfig::YawReachedMargin;

	//Set all neccesary publishers and subscribers
	_posPub = _pnh->advertise<geometry_msgs::Point32>(FSMConfig::PathPlannerPosTopic, 1);
	_obsPub = _pnh->advertise<ascend_msgs::PathPlannerPlan>(FSMConfig::PathPlannerObsTopic, 1);
	_targetPub = _pnh->advertise<geometry_msgs::Point32>(FSMConfig::PathPlannerTargetTopic , 1);
	_planSub = _pnh->subscribe(FSMConfig::PathPlannerPlanTopic, 1, &GoToState::pathRecievedCB, this);
	
}

//Calculates a yaw setpoints that is a multiple of 90 degrees
//and is as close to the path direction as possible 
//NOTE - method assumes dx and dy is not equal to zero
double GoToState::calculatePathYaw(double dx, double dy) {
	//Avoids division by zero
	if(std::fabs(dx + dy) < 0.001) {
		return 0;
	}
	/*
	angle = acos(([dx, dy] dot [1,0]) / (norm([dx, dy]) * norm([1,0]))) = acos(dx / (norm([dx, dy]) * 1))
	*/
	double angle = std::acos(dx / std::sqrt(dx * dx + dy * dy));

	if(angle > 3 * PI / 4) {
		angle = PI;
	} else if(angle > PI/4) {
		angle = PI/2.0;
	} else {
		angle = 0;
	}
	//
	if(dy < 0) {
		angle *= -1;
	}

	return angle;
}




