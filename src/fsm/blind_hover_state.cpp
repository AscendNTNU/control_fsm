#include "control/fsm/blind_hover_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <ros/ros.h>
#include <control/tools/target_tools.hpp>
#include <control/tools/logger.hpp>
#include "control/fsm/control_fsm.hpp"
#include "control/fsm/event_data.hpp"
#include "control/tools/config.hpp"

#define DEFAULT_BLIND_HOVER_ALTITUDE 1.0f    

/*
Only blind states (blind hover and blind land) will accept running without valid position.
*/

BlindHoverState::BlindHoverState() {
    setpoint_.type_mask = default_mask | IGNORE_PX | IGNORE_PY;
    setpoint_.position.z = DEFAULT_BLIND_HOVER_ALTITUDE;
}

void BlindHoverState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidRequest()) {
        if(event.request == RequestType::ABORT){
            if(cmd_.isValidCMD()) {
                control::handleInfoMsg("Aborting CMD");
                cmd_.eventError("ABORT");
                cmd_ = EventData();
            } else {
                control::handleWarnMsg("Can't abort blind hover");
            }
        } else {
            control::handleWarnMsg("Invalid transition request");
        }
    } else if(event.isValidCMD()) {
        if(!cmd_.isValidCMD()) {
            cmd_ = event; //Hold event until position is regained.
        } else {
            event.eventError("CMD rejected!");
            control::handleInfoMsg("ABORT old command first");
        }
    } else  {
        control::handleInfoMsg("Event ignored!");
    }
}

void BlindHoverState::stateBegin(ControlFSM& fsm, const EventData& event ) {
    ///Get shared_ptr to drones pose
    auto pose_p = control::Pose::getSharedPosePtr();
    //If full position is valid - no need to blind hover
    if(pose_p->isPoseValid()) {
        if(event.isValidCMD()) {
            fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, event); //Pass command on to next state
        } else {
            RequestEvent req_event(RequestType::POSHOLD);
            //Set target altitude if passed on from takeoff
            if(event.position_goal.z_valid) {
                req_event.position_goal = event.position_goal;
            }
            fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, req_event);
        }
        return;
    }
    if(event.isValidCMD()) {
        cmd_ = event;
    }

    setpoint_.position.z = control::Config::blind_hover_alt;
    setpoint_.yaw = control::getMavrosCorrectedTargetYaw(pose_p->getYaw());
}

void BlindHoverState::loopState(ControlFSM& fsm) {
    ///Get shared_ptr to drones pose
    auto pose_p = control::Pose::getSharedPosePtr();
    //Transition to position hold when position is valid.
    if(pose_p->isPoseValid()) {
        if(cmd_.isValidCMD()) {
            fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, cmd_);
            cmd_ = EventData(); //Reset cmd_
        } else {
            RequestEvent event(RequestType::POSHOLD);
            fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, event);
        }
    }
}

const mavros_msgs::PositionTarget* BlindHoverState::getSetpointPtr() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}


void BlindHoverState::handleManual(ControlFSM &fsm) {
    if(cmd_.isValidCMD()) {
        cmd_.eventError("Lost OFFBOARD");
        cmd_ = EventData();
    }
    RequestEvent manual_event(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUAL_FLIGHT_STATE, this, manual_event);
}
