#include "control_fsm/estimate_adjust_state.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/control_fsm.hpp"

EstimateAdjustState::EstimateAdjustState() {
    setpoint_.type_mask = default_mask | IGNORE_PX | IGNORE_PY; //State should transition before this is needed, but just in case
}

void EstimateAdjustState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidCMD()) {
        if(cmd_.isValidCMD()) {
            event.eventError("ABORT old CMD first!");
            fsm.handleFSMWarn("ABORT old CMD before sending new!");
        } else { 
            cmd_ = event;
        }
    } else if(event.event_type == EventType::REQUEST) {
        if(event.request == RequestType::ABORT && cmd_.isValidCMD()) {
            cmd_ = EventData();
            cmd_.eventError("ABORT request!");
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
    bool pos_invalid = true;

    if(pos_invalid) {
        if(cmd_.isValidCMD()) {
            fsm.transitionTo(ControlFSM::BLIND_HOVER_STATE, this, cmd_);
            cmd_ = EventData();
        } else {
            RequestEvent event(RequestType::BLINDHOVER);
            fsm.transitionTo(ControlFSM::BLIND_HOVER_STATE, this, event);
        }
    }
}

void EstimateAdjustState::stateBegin(ControlFSM& fsm, const EventData& event) {
    fsm.handleFSMError("EstimateAdjust has not been properly implemented!! Take manual control!");
    setpoint_.yaw = (float) fsm.getMavrosCorrectedYaw();

}

const mavros_msgs::PositionTarget* EstimateAdjustState::getSetpoint() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

void EstimateAdjustState::handleManual(ControlFSM &fsm) {
    fsm.handleFSMWarn("Lost OFFBOARD while adjusting position estimates! Do NOT switch back to OFFBOARD. Can lead to undefined behaviour!");
    //TODO Should it transition to MANUAL_FLIGHT_STATE?
}

