#include "control/fsm/position_hold_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <geometry_msgs/PoseStamped.h>
#include "control/fsm/control_fsm.hpp"
#include "control/fsm/event_data.hpp"
#include "control/tools/config.hpp"
#include <ros/ros.h>
#include <control/tools/target_tools.hpp>
#include <control/tools/logger.hpp>
#include <control/exceptions/pose_not_valid_exception.hpp>
#include "control/tools/drone_handler.hpp"


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
        if(event.command_type == CommandType::TAKEOFF) {
            event.finishCMD();
        } else {
            //All other command needs to go via the GOTO state
            fsm.transitionTo(ControlFSM::GO_TO_STATE, this, event);
        }
    }
}

void PositionHoldState::stateInit(ControlFSM& fsm) {
    std::function<void()> obstacleAvoidanceCB = [this]()->void {
        //TODO Why not set position setpoint to current position here??
        this->obstacle_avoidance_kicked_in_ = true;
    };

    fsm.obstacle_avoidance_.registerOnWarnCBPtr(std::make_shared< std::function<void()> >(obstacleAvoidanceCB));

    // Set flag for obstacle avoidance
    obstacle_avoidance_kicked_in_ = false;

}

void PositionHoldState::stateBegin(ControlFSM& fsm, const EventData& event) {
    //No need to check other commands
    if(event.isValidCMD()) {
        //All valid commands need to go to correct place on arena before anything else
        fsm.transitionTo(ControlFSM::GO_TO_STATE, this, event);
        return;
    }
    try {
        if(!control::DroneHandler::isPoseValid()) {
            //Error if pose is not valid
            throw control::PoseNotValidException();
        }

        // Set flag for obstacle avoidance
        obstacle_avoidance_kicked_in_ = false;


        //Get pose - and position
        auto pose_stamped = control::DroneHandler::getCurrentPose();
        auto& position = pose_stamped.pose.position;
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
        setpoint_.position.z = position.z;
        //Set setpoint altitude to position_goal if valid
        if(event.position_goal.z_valid) {
            //Set z setpoint to position_goal if we're close enough for it to be safe
            if(std::fabs(position.z - event.position_goal.z) <= control::Config::altitude_reached_margin) {
                setpoint_.position.z = event.position_goal.z;
            }
        }
        //If altitude is too low - set it to minimum altitude
        if(setpoint_.position.z < control::Config::min_in_air_alt) {
            control::handleWarnMsg("Poshold target altitude too low, using min altitude");
            setpoint_.position.z = control::Config::min_in_air_alt;
        }

        using control::pose::quat2yaw;
        using control::getMavrosCorrectedTargetYaw;
        auto& quat = pose_stamped.pose.orientation;
        //Set yaw target
        setpoint_.yaw = static_cast<float>(getMavrosCorrectedTargetYaw(quat2yaw(quat)));
    } catch (const std::exception& e){
        //Transition to blindhover if pose not valid
        control::handleCriticalMsg(e.what());
        //No need to handle commands - not recoverable
        EventData n_event;
        n_event.event_type = EventType::POSLOST;
        fsm.transitionTo(ControlFSM::BLIND_HOVER_STATE, this, n_event);
        return;
    }
}

void PositionHoldState::loopState(ControlFSM& fsm){
    if (obstacle_avoidance_kicked_in_){
	//TODO: logic to avoid being pushed around
	
        // keep new setpoint after obstacle avoidance
	auto pose_stamped = control::DroneHandler::getCurrentPose();
        auto& position = pose_stamped.pose.position;
        //Set setpoint to current position
        setpoint_.position.x = position.x;
        setpoint_.position.y = position.y;
    }
    //Only react once
    obstacle_avoidance_kicked_in_ = false;
}

void PositionHoldState::stateEnd(ControlFSM& fsm, const EventData& event) {
    // Set flag for obstacle avoidance
    obstacle_avoidance_kicked_in_ = false;
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


ascend_msgs::ControlFSMState PositionHoldState::getStateMsg() const {
    using ascend_msgs::ControlFSMState;
    ControlFSMState msg;
    msg.name = getStateName();
    msg.state_id = ControlFSMState::POS_HOLD_STATE;
    return msg;
}

bool PositionHoldState::stateIsReady(ControlFSM& fsm){
    return true;
}
