#include "control_fsm/blind_hover_state.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/control_fsm.hpp"
#include "control_fsm/event_data.hpp"
#include "control_fsm/fsm_config.hpp"

#define DEFAULT_BLIND_HOVER_ALTITUDE 1.0f    

/*
Only blind states (blind hover and blind land) will accept running without valid position.
*/

BlindHoverState::BlindHoverState() {
    setpoint_.type_mask = default_mask | IGNORE_PX | IGNORE_PY;
    setpoint_.position.z = DEFAULT_BLIND_HOVER_ALTITUDE;
}

void BlindHoverState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidCMD()) {
        if(!cmd_.isValidCMD()) {
            cmd_ = event; //Hold event until position is regained.
        } else {
            event.eventError("CMD rejected!");
            fsm.handleFSMWarn("ABORT old command first");
        }
    } else if(event.isValidRequest()) {
        if(event.request == RequestType::BLINDLAND) {
            if(cmd_.isValidCMD()) {
                cmd_.eventError("Manual request overriding cmd");
                cmd_ = EventData();
            }
            fsm.transitionTo(ControlFSM::BLINDLANDSTATE, this, event);
        } else if(event.request == RequestType::ABORT){
            if(cmd_.isValidCMD()) {
                fsm.handleFSMInfo("Aborting CMD");
                cmd_.eventError("ABORT");
                cmd_ = EventData();
            } else {
                fsm.handleFSMWarn("Can't abort blind hover");
            }
        } else {
            fsm.handleFSMWarn("Invalid transition request");
        }
    } else {
        fsm.handleFSMInfo("Event ignored!");
    }
}

void BlindHoverState::stateBegin(ControlFSM& fsm, const EventData& event ) {
    //If full position is valid - no need to blind hover
    if(fsm.getPositionXYZ() != nullptr) {
        if(event.isValidCMD()) {
            fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event); //Pass command on to next state
        } else {
            RequestEvent rEvent(RequestType::POSHOLD);
            fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, rEvent);
        }
        return;
    }
    if(event.isValidCMD()) {
        cmd_ = event;
    }

    setpoint_.position.z = FSMConfig::BlindHoverAlt;
    setpoint_.yaw = fsm.getMavrosCorrectedYaw();
}

void BlindHoverState::loopState(ControlFSM& fsm) {
    //Transition to position hold when position is valid.
    if(fsm.getPositionXYZ() != nullptr) {
        if(cmd_.isValidCMD()) {
            fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, cmd_);
            cmd_ = EventData(); //Reset cmd_
        } else {
            RequestEvent event(RequestType::POSHOLD);
            fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event);
        }
    }
}

const mavros_msgs::PositionTarget* BlindHoverState::getSetpoint() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}


void BlindHoverState::handleManual(ControlFSM &fsm) {
    if(cmd_.isValidCMD()) {
        cmd_.eventError("Lost OFFBOARD");
        cmd_ = EventData();
    }
    RequestEvent manualEvent(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUALFLIGHTSTATE, this, manualEvent);
}