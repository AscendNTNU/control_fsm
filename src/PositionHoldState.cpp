#include "control_fsm/PositionHoldState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <geometry_msgs/PoseStamped.h>
#include "control_fsm/ControlFSM.hpp"
#include "control_fsm/EventData.hpp"
#include "control_fsm/FSMConfig.hpp"
#include <ascend_msgs/PointArray.h>
#include <ros/ros.h>


//Constructor sets default setpoint type mask
PositionHoldState::PositionHoldState() {
	_setpoint.type_mask = default_mask;
}

//Handles incoming events
void PositionHoldState::handleEvent(ControlFSM& fsm, const EventData& event) {
	_isActive = true;
	if(event.isValidCMD()) {
		//All valid command needs to go via the GOTO state
		fsm.transitionTo(ControlFSM::GOTOSTATE, this, event);
	} else if(event.isValidRequest()) {
		switch(event.request) {
			case RequestType::GOTO:
				fsm.transitionTo(ControlFSM::GOTOSTATE, this, event);
				break;
			case RequestType::LAND:
				fsm.transitionTo(ControlFSM::LANDSTATE, this, event);
				break;
			case RequestType::BLINDLAND:
				fsm.transitionTo(ControlFSM::BLINDLANDSTATE, this, event);
				break;

			/*case RequestType::TRACKGB:
				fsm.transitionTo(ControlFSM::TRACKGBSTATE, this, event);
				break;
			*/
			 case RequestType::ESTIMATORADJ:
				fsm.transitionTo(ControlFSM::ESTIMATEADJUSTSTATE, this, event);
				break;
			default:
				fsm.handleFSMWarn("Transition not allowed");
				break;
		}
	} else if(event.eventType == EventType::POSLOST) {
		fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, event);
	} else if(event.eventType == EventType::AUTONOMOUS) {
		fsm.transitionTo(*this, this, event); //Transition to itself, sets correct setpoint after manual mode
		/*
		TODO This might not be a suffecient solution to midflight loss of OFFBOARD. 
		If the mavros state message arrives "late" the drone will try to go to the old
		position setpoints. Consider adding another state for manual flight (that always use current position as setpoint)
		*/
	}
}

void PositionHoldState::stateBegin(ControlFSM& fsm, const EventData& event) {
	if(_pFsm == nullptr) {
		_pFsm = &fsm;
	}
	_safeHoverAlt = FSMConfig::SafeHoverAltitude;
	//No need to check other commands
	if(event.isValidCMD()) {
		//All valid commands need to go to correct place on arena before anything else
		fsm.transitionTo(ControlFSM::GOTOSTATE, this, event); 
	}

	const geometry_msgs::PoseStamped* pose = fsm.getPositionXYZ();
	//GoTo blind hover if position not valid, should never occur
	if(pose == nullptr) {
		if(event.isValidCMD()) {
			event.eventError("No valid position!");
		}
		EventData nEvent;
		nEvent.eventType = EventType::POSLOST;
		fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, nEvent); 
		return;
	}

	//Set setpoint to current position
	_setpoint.position.x = pose->pose.position.x;
	_setpoint.position.y = pose->pose.position.y;
	//Keep old altitude if abort
	//TODO Should we use an default hover altitude in case of ABORT?
	if(!event.isValidRequest() || event.request != RequestType::ABORT) {
		_setpoint.position.z = pose->pose.position.z;
	}

	_setpoint.yaw = (float) fsm.getMavrosCorrectedYaw();
	
}

void PositionHoldState::stateInit(ControlFSM &fsm) {
	_pFsm = &fsm;
	_lidarSub = fsm._pnh->subscribe(FSMConfig::LidarTopic, 1, &PositionHoldState::obsCB, this);
}

bool PositionHoldState::stateIsReady() {
	//Skipping check is allowed in debug mode
	if(!FSMConfig::RequireAllDataStreams) return true;
	return _lidarSub.getNumPublishers() > 0;
}

void PositionHoldState::obsCB(const ascend_msgs::PointArray::ConstPtr& msg) {
	//TODO TEST!!!
	//Only check if neccesary
	if(!_isActive || _setpoint.position.z >= _safeHoverAlt) {
		return;
	}
	if(_pFsm == nullptr) {
		ROS_ERROR("FSM pointer = nullptr! Critical!");
		return; //Avoids nullpointer exception
	}
	auto points = msg->points;
	const geometry_msgs::PoseStamped* pPose = _pFsm->getPositionXYZ();
	//Should never happen!
	if(pPose == nullptr) {
		//No valid XY position available, no way to determine distance to GB
		_pFsm->handleFSMError("Position not available! Should not happen!");
		return;
	}
	//No need to check obstacles if they're too close
	if(pPose->pose.position.z >= FSMConfig::SafeHoverAltitude) {
		return;
	}

	double droneX = pPose->pose.position.x;
	double droneY = pPose->pose.position.y;
	for(int i = 0; i < points.size(); ++i) {
		double distSquared = std::pow(droneX - points[i].x, 2) + std::pow(droneY - points[i].y, 2);
		if(distSquared < std::pow(FSMConfig::ObstacleTooCloseDist, 2)) {
			_setpoint.position.z = _safeHoverAlt;
			return; //No need to check the rest!
		}
	}


}

void PositionHoldState::stateEnd(ControlFSM &fsm, const EventData& eventData) {
	_isActive = false;
}

//Returns setpoint
const mavros_msgs::PositionTarget* PositionHoldState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint;
}