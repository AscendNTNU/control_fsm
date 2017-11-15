#include "control/fsm/takeoff_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control/fsm/control_fsm.hpp"
#include <cmath>
#include <string>
#include <control/tools/target_tools.hpp>
#include <control/tools/logger.hpp>
#include "control/tools/config.hpp"

TakeoffState::TakeoffState() {
    setpoint_ = mavros_msgs::PositionTarget();
    //Ignoring PX and PY, takeoff without XY feedback
    setpoint_.type_mask = default_mask | SETPOINT_TYPE_TAKEOFF | IGNORE_PX | IGNORE_PY;
    setpoint_.position.z = DEFAULT_TAKEOFF_ALTITUDE;
}

void TakeoffState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidRequest()) {
        if(event.request == RequestType::ABORT && cmd_.isValidCMD()) {
            cmd_.eventError("Aborting command");
            cmd_ = EventData(); //Aborting commands, but will still continue takeoff
            control::handleInfoMsg("Command aborted, but takeoff can't be aborted");
        } else {
            control::handleWarnMsg("Illegal transition request");
        }
    } else if(event.isValidCMD()) {
        if(cmd_.isValidCMD()) {
            cmd_ = event;
        } else {
            event.eventError("Finish old CMD before sending new");
            control::handleWarnMsg("ABORT old cmd before sending new");
        }
    } else {
        control::handleInfoMsg("Event ignored");
    }
}

void TakeoffState::stateBegin(ControlFSM& fsm, const EventData& event) {
    //Set relevant parameters
    using control::Config;
    //Takeoff altitude
    setpoint_.position.z = Config::takeoff_altitude;
    //Takeoff finished threshold
    altitude_reached_margin_ = Config::altitude_reached_margin;

    if(event.isValidCMD()) {
        cmd_ = event;
        cmd_.sendFeedback("Takeoff!!");
    }
    auto pose_p = control::Pose::getSharedPosePtr();
    control::Point current_position = pose_p->getPositionXYZ();
    //If no position is available - abort takeoff
    if(!pose_p->isPoseValid()) {
        control::handleErrorMsg("No position available");
        if(cmd_.isValidCMD()) {
            cmd_.eventError("No position available");
            cmd_ = EventData();
        }
        RequestEvent abort_event(RequestType::ABORT);
        fsm.transitionTo(ControlFSM::IDLE_STATE, this, abort_event);
        return;
    }
    //Set yaw setpoint based on current rotation
    setpoint_.yaw = control::getMavrosCorrectedTargetYaw(pose_p->getYaw());
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
            //Pass on altitude target
            event.position_goal = PositionGoal(setpoint_.position.z);
            fsm.transitionTo(ControlFSM::BLIND_HOVER_STATE, this, event);
        }
    }
}

const mavros_msgs::PositionTarget* TakeoffState::getSetpointPtr() {
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

