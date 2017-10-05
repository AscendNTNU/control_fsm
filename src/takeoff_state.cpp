#include "control_fsm/takeoff_state.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/control_fsm.hpp"
#include <cmath>
#include <string>
#include "control_fsm/fsm_config.hpp"

TakeoffState::TakeoffState() {
    setpoint_ = mavros_msgs::PositionTarget();
    setpoint_.type_mask = default_mask | SETPOINT_TYPE_TAKEOFF;
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
    setpoint_.position.z = FSMConfig::TakeoffAltitude;
    //Takeoff finished threshold
    altitude_reached_margin_ = FSMConfig::AltitudeReachedMargin;

    if(event.isValidCMD()) {
        cmd_ = event;
        cmd_.sendFeedback("Takeoff!!");
    }
    const geometry_msgs::PoseStamped* pose = fsm.getPositionXYZ();
    //If no position is available - abort takeoff
    if(pose == nullptr) {
        fsm.handleFSMError("No position available");
        if(cmd_.isValidCMD()) {
            cmd_.eventError("No position available");
            cmd_ = EventData();
        }
        RequestEvent abortEvent(RequestType::ABORT);
        fsm.transitionTo(ControlFSM::IDLESTATE, this, abortEvent);
        return;
    }
    //Set takeoff setpoint to current XY position
    setpoint_.position.x = pose->pose.position.x;
    setpoint_.position.y = pose->pose.position.y;
    //Set yaw setpoint based on current rotation
    setpoint_.yaw = (float) fsm.getMavrosCorrectedYaw();;
}

void TakeoffState::loopState(ControlFSM& fsm) {
    double z = fsm.getPositionZ();
    if(z > (setpoint_.position.z - altitude_reached_margin_)) {
        if(cmd_.isValidCMD()) {
            fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, cmd_);
            cmd_ = EventData();
        } else {
            RequestEvent event(RequestType::BLINDHOVER);
            fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, event);
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
    RequestEvent manualEvent(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUALFLIGHTSTATE, this, manualEvent);
}

