#include "control_fsm/PreFlightState.hpp"
#include "control_fsm/ControlFSM.hpp"

//Only check for an abort event
void PreFlightState::handleEvent(ControlFSM& fsm, const EventData& event) {
	if(event.request == RequestType::ABORT) {
		//Transition back to start
		fsm.transitionTo(ControlFSM::BEGINSTATE, this, event);
	} else {
		fsm.handleFSMInfo("Invalid transiton");
	}
}

//Returns setpoint
const mavros_msgs::PositionTarget& PreFlightState::getSetpoint() { 
	_setpoint.header.stamp = ros::Time::now();
	return _setpoint; 
}



