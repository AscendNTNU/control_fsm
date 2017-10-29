#include "control_fsm/position_hold_state.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <geometry_msgs/PoseStamped.h>
#include "control_fsm/control_fsm.hpp"
#include "control_fsm/event_data.hpp"
#include "control_fsm/fsm_config.hpp"
#include <ascend_msgs/PointArray.h>
#include <ros/ros.h>
#include <control_fsm/tools/target_tools.hpp>


//Constructor sets default setpoint type mask
PositionHoldState::PositionHoldState() {
    setpoint_.type_mask = default_mask;
}

//Handles incoming events
void PositionHoldState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidRequest()) {
        switch(event.request) {
            case RequestType::GOTO:
                fsm.transitionTo(ControlFSM::GO_TO_STATE, this, event);
                break;
            case RequestType::LAND:
                fsm.transitionTo(ControlFSM::LAND_STATE, this, event);
                break;
            /*case RequestType::TRACKGB:
                fsm.transitionTo(ControlFSM::TRACK_GB_STATE, this, event);
                break;
            */
            default:
                fsm.handleFSMWarn("Transition not allowed");
                break;
        }
    } else if(event.isValidCMD()) {
        //All valid command needs to go via the GOTO state
        fsm.transitionTo(ControlFSM::GO_TO_STATE, this, event);
    }
}

void PositionHoldState::stateBegin(ControlFSM& fsm, const EventData& event) {
    if(fsm_p_ == nullptr) {
        fsm_p_ = &fsm;
    }
    //No need to check other commands
    if(event.isValidCMD()) {
        //All valid commands need to go to correct place on arena before anything else
        fsm.transitionTo(ControlFSM::GO_TO_STATE, this, event);
        return;
    }

    auto pose_p = control::Pose::getSharedPosePtr();
    control::Point position = pose_p->getPositionXYZ();
    //GoTo blind hover if position not valid, should never occur
    if(!pose_p->isPoseValid()) {
        if(event.isValidCMD()) {
            event.eventError("No valid position!");
        }
        EventData n_event;
        n_event.event_type = EventType::POSLOST;
        fsm.transitionTo(ControlFSM::BLIND_HOVER_STATE, this, n_event);
        return;
    }

    //Set setpoint to current position
    setpoint_.position.x = position.x;
    setpoint_.position.y = position.y;
    //Keep old altitude if abort
    //TODO Should we use an default hover altitude in case of ABORT?
    if(!event.isValidRequest() || event.request != RequestType::ABORT) {
        setpoint_.position.z = position.z;
    }

    setpoint_.yaw = control::getMavrosCorrectedTargetYaw(pose_p->getYaw());
}

void PositionHoldState::stateInit(ControlFSM &fsm) {
    fsm_p_ = &fsm;
}

bool PositionHoldState::stateIsReady(ControlFSM &fsm) {

    //Return true if obstacle detection is disabled
    if(!FSMConfig::require_obstacle_detection) return true;

    //Skipping check is allowed in debug mode
    if(!FSMConfig::require_all_data_streams) return true;
    return true;
}


void PositionHoldState::stateEnd(ControlFSM &fsm, const EventData& eventData) {
}

//Returns setpoint
const mavros_msgs::PositionTarget* PositionHoldState::getSetpoint() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

void PositionHoldState::handleManual(ControlFSM &fsm) {
    RequestEvent manual_event(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUAL_FLIGHT_STATE, this, manual_event);
}
