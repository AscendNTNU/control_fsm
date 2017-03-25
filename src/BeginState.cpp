#include "control_fsm/BeginState.hpp"
#include "control_fsm/ControlFSM.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>

//Begin state only waits for preflight request
void BeginState::handleEvent(ControlFSM& fsm, const EventData& event) {
	if(event.request == RequestType::PREFLIGHT) {
		fsm.transitionTo(ControlFSM::PREFLIGHTSTATE, this, event);
	} else {
		fsm.handleFSMInfo("Invalid transiton");
	}
}

//Returns IDLE setpoints - nothing will though happen as drone is disarmed
const mavros_msgs::PositionTarget& BeginState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	_setpoint.type_mask = default_mask | SETPOINT_TYPE_IDLE;
 	return _setpoint;
}



