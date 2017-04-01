#include "control_fsm/PositionHoldState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <geometry_msgs/PoseStamped.h>
#include "control_fsm/ControlFSM.hpp"
//Constructor sets default setpoint type mask
PositionHoldState::PositionHoldState() {
	_setpoint.type_mask = default_mask;
}

//Handles incoming events
void PositionHoldState::handleEvent(ControlFSM& fsm, const EventData& event) {
	if(event.eventType == EventType::COMMAND) {
		switch(event.commandType) {
			case CommandType::GOTOXYZ:
			case CommandType::LANDXY:
				fsm.transitionTo(ControlFSM::GOTOSTATE, this, event); //Transition on to GOTO state
				break;
			case CommandType::LANDGB:
				fsm.transitionTo(ControlFSM::TRACKGBSTATE, this, event);
				break;
			default:
				break;
		}
	} else if(event.eventType == EventType::REQUEST) {
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
			case RequestType::TRACKGB:
				fsm.transitionTo(ControlFSM::TRACKGBSTATE, this, event);
				break;
			case RequestType::ESTIMATORADJ:
				fsm.transitionTo(ControlFSM::ESTIMATEADJUSTSTATE, this, event);
				break;
			case RequestType::NONE:
				break; //Does nothin
			default:
				fsm.handleFSMWarn("Transition not allowed");
				break;
		}
	} else if(event.eventType == EventType::POSLOST) {
		fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, event);
	}
}

void PositionHoldState::stateBegin(ControlFSM& fsm, const EventData& event) {
	const geometry_msgs::PoseStamped* pose = fsm.getPosition();
	//GoTo blind hover if position not valid
	if(pose == nullptr) {
		EventData nEvent = event;
		nEvent.eventType = EventType::POSLOST;
		nEvent.request = RequestType::BLINDHOVER;
		fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, nEvent); 
		return;
	}

	//Set setpoint to current position
	_setpoint.position.x = pose->pose.position.x;
	_setpoint.position.y = pose->pose.position.y;
	_setpoint.position.z = pose->pose.position.z;

	//No need to check other commands
	if(event.eventType != EventType::COMMAND || event.commandType == CommandType::NONE) {
		return; 
	}
	//Check command and make correct transition
	switch(event.commandType) {
		case CommandType::GOTOXYZ:
		case CommandType::LANDXY:
			fsm.transitionTo(ControlFSM::GOTOSTATE, this, event); //Transition on to GOTO state
			break;
		case CommandType::LANDGB:
			fsm.transitionTo(ControlFSM::TRACKGBSTATE, this, event);
			break;
		default:
			break;
	}
}

//Returns setpoint
const mavros_msgs::PositionTarget* PositionHoldState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint;
}