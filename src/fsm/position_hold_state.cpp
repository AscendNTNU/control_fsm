#include "control/fsm/position_hold_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <geometry_msgs/PoseStamped.h>
#include "control/fsm/control_fsm.hpp"
#include "control/fsm/event_data.hpp"
#include "control/tools/config.hpp"
#include <ascend_msgs/PointArray.h>
#include <ros/ros.h>
#include <control/tools/target_tools.hpp>
#include <control/tools/logger.hpp>


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
                control::handleWarnMsg("Transition not allowed");
                break;
        }
    } else if(event.isValidCMD()) {
        //All valid command needs to go via the GOTO state
        fsm.transitionTo(ControlFSM::GO_TO_STATE, this, event);
    }
}

void PositionHoldState::stateBegin(ControlFSM& fsm, const EventData& event) {
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

    //If positiongoal is valid, but it's not a command
    if(event.position_goal.xyz_valid) {
        //Set xy setpoint to positionGoal if we're close enough for it to be safe
        double xy_dist_square = std::pow(position.x - event.position_goal.x, 2) + std::pow(position.y - event.position_goal.y, 2);
        if(xy_dist_square <= std::pow(control::Config::setpoint_reached_margin, 2)) {
            setpoint_.position.x = event.position_goal.x;
            setpoint_.position.y = event.position_goal.y;
        }
    }

    //Keep old altitude if abort
    //TODO Should we use an default hover altitude in case of ABORT?
    if(!event.isValidRequest() || event.request != RequestType::ABORT) {
        setpoint_.position.z = position.z;
        //Set setpoint altitude to position_goal if valid
        if(event.position_goal.z_valid) {
            //Set z setpoint to position_goal if we're close enough for it to be safe
            if(std::fabs(position.z - event.position_goal.z) <= control::Config::altitude_reached_margin) {
                setpoint_.position.z = event.position_goal.z;
            }
        }
    }
    setpoint_.yaw = static_cast<float>(control::getMavrosCorrectedTargetYaw(pose_p->getYaw()));
}

//Returns setpoint
const mavros_msgs::PositionTarget* PositionHoldState::getSetpointPtr() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

void PositionHoldState::handleManual(ControlFSM &fsm) {
    RequestEvent manual_event(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUAL_FLIGHT_STATE, this, manual_event);
}
