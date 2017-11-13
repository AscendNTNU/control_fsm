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
#include <control/exceptions/PoseNotValidException.hpp>


//Constructor sets default setpoint type mask
PositionHoldState::PositionHoldState() {
    setpoint_.type_mask = default_mask;
}

//Handles incoming events
void PositionHoldState::handleEvent(ControlFSM& fsm, const EventData& event) {
    is_active_ = true;
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
    if(fsm_p_ == nullptr) {
        fsm_p_ = &fsm;
    }
    safe_hover_alt_ = control::Config::safe_hover_altitude;
    //No need to check other commands
    if(event.isValidCMD()) {
        //All valid commands need to go to correct place on arena before anything else
        fsm.transitionTo(ControlFSM::GO_TO_STATE, this, event);
        return;
    }
    //Get pose
    std::shared_ptr<control::Pose> pose_p;
    try {
        pose_p = control::Pose::getSharedPosePtr();
        if(!pose_p->isPoseValid()) {
            throw control::PoseNotValidException();
        }
    } catch (const std::exception& e){
        //Transition to blindhover if pose not valid
        control::handleCriticalMsg(e.what());
        //No need to handle commands - not recoverable
        EventData n_event;
        n_event.event_type = EventType::POSLOST;
        fsm.transitionTo(ControlFSM::BLIND_HOVER_STATE, this, n_event);
        return;
    }
    //Get position
    control::Point position = pose_p->getPositionXYZ();
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

void PositionHoldState::stateInit(ControlFSM &fsm) {
    fsm_p_ = &fsm;
    lidar_sub_ = fsm.node_handler_.subscribe(control::Config::lidar_topic, 1, &PositionHoldState::obsCB, this);
}

bool PositionHoldState::stateIsReady(ControlFSM &fsm) {
    using control::Config;
    //Return true if obstacle detection is disabled
    if(!Config::require_obstacle_detection) return true;

    //Skipping check is allowed in debug mode
    if(!Config::require_all_data_streams) return true;

    if(lidar_sub_.getNumPublishers() > 0) {
        return true;
    } else {
        control::handleWarnMsg("No lidar publisher in posHold");
        return false;
    }
}

//Will be removed!
void PositionHoldState::obsCB(const ascend_msgs::PointArray::ConstPtr& msg) {
    using control::Config;
    //TODO TEST!!!
    //Only check if neccesary
    if(!is_active_ || setpoint_.position.z >= safe_hover_alt_) {
        return;
    }
    if(fsm_p_ == nullptr) {
        control::handleErrorMsg("FSM pointer = nullptr! Critical!");
        return; //Avoids nullpointer exception
    }
    auto points = msg->points;
    auto pose_p = control::Pose::getSharedPosePtr();
    control::Point position = pose_p->getPositionXYZ();
    //Should never happen!
    if(pose_p == nullptr) {
        //No valid XY position available, no way to determine distance to GB
        control::handleWarnMsg("Position not available! Should not happen!");
        return;
    }
    //No need to check obstacles if we're high enough
    if(position.z >= Config::safe_hover_altitude) {
        return;
    }

    double drone_x = position.x;
    double drone_y = position.y;
    for(unsigned int i = 0; i < points.size(); ++i) {
        double dist_squared = std::pow(drone_x - points[i].x, 2) + std::pow(drone_y - points[i].y, 2);
        if(dist_squared < std::pow(Config::obstacle_too_close_dist, 2)) {
            setpoint_.position.z = safe_hover_alt_;
            return; //No need to check the rest!
        }
    }


}

void PositionHoldState::stateEnd(ControlFSM &fsm, const EventData& eventData) {
    is_active_ = false;
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
