#include "control/fsm/track_gb_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <ros/ros.h>
#include <control/fsm/control_fsm.hpp>
#include <control/tools/logger.hpp>

TrackGBState::TrackGBState() {
    setpoint_.type_mask = default_mask;
}

void TrackGBState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidRequest()) {
        if(event.request == RequestType::ABORT) {
            if(cmd_.isValidCMD()) {
                cmd_.eventError("ABORT");
                cmd_ = EventData();
            }
            fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, event);
        } else if(event.request == RequestType::POSHOLD) {
            if(cmd_.isValidCMD()) {
                control::handleWarnMsg("ABORT CMD first!");
            } else {
                fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, event);
            }
        }
    } else if(event.isValidCMD()) {
        if(cmd_.isValidCMD()) {
            control::handleWarnMsg("ABORT cmd before sending new");
        } else {
            if(event.command_type == CommandType::TAKEOFF) {
                //Drone already in air!
                control::handleWarnMsg("Wrong usage of takeoff cmd, already in air!");
                event.finishCMD();
            } else {
                //Transition to poshold, using new command
                fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, event);
            }
        }
    }
}

void TrackGBState::stateBegin(ControlFSM& fsm, const EventData& event) {
    cmd_ = event;
}

void TrackGBState::loopState(ControlFSM& fsm) {
    //TODO Implement ground robot tracking
}

const mavros_msgs::PositionTarget* TrackGBState::getSetpointPtr() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

void TrackGBState::handleManual(ControlFSM &fsm) {
    if(cmd_.isValidCMD()) {
        cmd_.eventError("Lost OFFBOARD");
        cmd_ = EventData();
    }
    RequestEvent manual_event(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUAL_FLIGHT_STATE, this, manual_event);
}
