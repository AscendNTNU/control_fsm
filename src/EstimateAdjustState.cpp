#include "control_fsm/EstimateAdjustState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/ControlFSM.hpp"

EstimateAdjustState::EstimateAdjustState() {
	_setpoint.type_mask = default_mask | IGNORE_PX | IGNORE_PY; //State should transition before this is needed, but just in case
}

void EstimateAdjustState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidCMD()) {
        if(_cmd.isValidCMD()) {
            event.eventError("ABORT old CMD first!");
            fsm.handleFSMWarn("ABORT old CMD before sending new!");
        } else { 
            _cmd = event;
        }
    } else if(event.eventType == EventType::REQUEST) {
        if(event.request == RequestType::ABORT && _cmd.isValidCMD()) {
            _cmd = EventData();
            _cmd.eventError("ABORT request!");
            fsm.handleFSMDebug("ABORTING command, but estimateadjust cant be aborted!");
        } else {
            fsm.handleFSMWarn("Illegal transition request");
        }
    } else {
        fsm.handleFSMDebug("Ignoring event");
    }
}

void EstimateAdjustState::loopState(ControlFSM& fsm) {
    //TODO Transition to blindhover as soon as position is invalid
    bool posInvalid = true;

    if(posInvalid) {
		if(_cmd.isValidCMD()) {
			fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, _cmd);
			_cmd = EventData();
		} else {
			RequestEvent event(RequestType::BLINDHOVER);
			fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, event);
		}
	}
}

void EstimateAdjustState::stateBegin(ControlFSM& fsm, const EventData& event) {
	_setpoint.yaw = (float) fsm.getMavrosCorrectedYaw();

}

const mavros_msgs::PositionTarget* EstimateAdjustState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint;
}

void EstimateAdjustState::handleManual(ControlFSM &fsm) {
    fsm.handleFSMWarn("Lost OFFBOARD while adjusting position estimates! Do NOT switch back to OFFBOARD. Can lead to undefined behaviour!");
    //TODO Should it transition to MANUALFLIGHTSTATE?
}

