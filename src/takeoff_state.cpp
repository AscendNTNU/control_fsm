#include "control_fsm/takeoff_state.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/control_fsm.hpp"
#include <cmath>
#include <string>
#include "control_fsm/fsm_config.hpp"

TakeoffState::TakeoffState() {
    setpoint_ = mavros_msgs::PositionTarget();
    //Ignoring PX and PY, takeoff without XY feedback
    setpoint_.type_mask = default_mask | SETPOINT_TYPE_TAKEOFF | IGNORE_PX | IGNORE_PY;
    setpoint_.position.z = DEFAULT_TAKEOFF_ALTITUDE;
}

void TakeoffState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidCMD()) {
        if(cmd_.isValidCMD()) {
            cmd_ = event;
        } else {
            event.eventError("Finish old CMD before sending new");
            fsm.handleFSMWarn("ABORT old cmd before sending new");
        }
    } else if(event.isValidRequest()) {
        if(event.request == RequestType::ABORT && cmd_.isValidCMD()) {
            cmd_.eventError("Aborting command");
            cmd_ = EventData(); //Aborting commands, but will still continue takeoff
            fsm.handleFSMInfo("Command aborted, but takeoff can't be aborted");
        } else {
            fsm.handleFSMWarn("Illegal transition request");
        }
    } else {
        fsm.handleFSMInfo("Event ignored");
    }
}

void TakeoffState::stateBegin(ControlFSM& fsm, const EventData& event) {
    //Set relevant parameters
    //Takeoff altitude
    setpoint_.position.z = FSMConfig::takeoff_altitude;
    //Takeoff finished threshold
    altitude_reached_margin_ = FSMConfig::altitude_reached_margin;

    if(event.isValidCMD()) {
        cmd_ = event;
        cmd_.sendFeedback("Takeoff!!");
    }
    auto pose_p = control::Pose::getSharedPosePtr();
    control::Point current_position = pose_p->getPositionXYZ();
    //If no position is available - abort takeoff
    if(!pose_p->isPoseValid()) {
        fsm.handleFSMError("No position available");
        if(cmd_.isValidCMD()) {
            cmd_.eventError("No position available");
            cmd_ = EventData();
        }
        RequestEvent abort_event(RequestType::ABORT);
        fsm.transitionTo(ControlFSM::IDLE_STATE, this, abort_event);
        return;
    }
    //Set yaw setpoint based on current rotation
    setpoint_.yaw = pose_p->getMavrosCorrectedYaw();
}

void TakeoffState::loopState(ControlFSM& fsm) {
    auto pose_p = control::Pose::getSharedPosePtr();
    control::Point current_position = pose_p->getPositionXYZ();
    if(current_position.z > (setpoint_.position.z - altitude_reached_margin_)) {
        if(cmd_.isValidCMD()) {
            fsm.transitionTo(ControlFSM::BLIND_HOVER_STATE, this, cmd_);
            cmd_ = EventData();
        } else {
            RequestEvent event(RequestType::BLINDHOVER);
            fsm.transitionTo(ControlFSM::BLIND_HOVER_STATE, this, event);
        }
    }
}

const mavros_msgs::PositionTarget* TakeoffState::getSetpoint() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

void TakeoffState::handleManual(ControlFSM &fsm) {
    if(cmd_.isValidCMD()) {
        cmd_.eventError("Lost OFFBOARD!");
        cmd_ = EventData();
    }
    RequestEvent manual_event(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUAL_FLIGHT_STATE, this, manual_event);
}

