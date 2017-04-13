#include "control_fsm/EstimateAdjustState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/ControlFSM.hpp"

EstimateAdjustState::EstimateAdjustState() {
	_setpoint.type_mask = default_mask | IGNORE_PX | IGNORE_PY; //State should transition before this is needed, but just in case
}

void EstimateAdjustState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidCMD()) {
        _cmd = event;
    } else if(event.eventType == EventType::REQUEST) {
        if(event.request == RequestType::ABORT && _cmd.eventType == EventType::COMMAND) {
            _cmd = EventData();
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
        if(_cmd.eventType == EventType::COMMAND) {
            fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, _cmd);
            _cmd = EventData();
        } else {
            EventData event;
            event.eventType == EventType::REQUEST;
            event.request == RequestType::BLINDHOVER;
            fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, event);
        }
    }
}

void EstimateAdjustState::stateBegin(ControlFSM& fsm, const EventData& event) {
	if(event.eventType == EventType::COMMAND && event.commandType != CommandType::NONE) {
        _cmd = event;
    }
}

const mavros_msgs::PositionTarget* EstimateAdjustState::getSetpoint() {
	_setpoint.header.stamp = ros::Time::now();
	return &_setpoint;
}

