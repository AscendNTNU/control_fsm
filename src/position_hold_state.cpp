#include "control_fsm/position_hold_state.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <geometry_msgs/PoseStamped.h>
#include "control_fsm/control_fsm.hpp"
#include "control_fsm/event_data.hpp"
#include "control_fsm/fsm_config.hpp"
#include <ascend_msgs/PointArray.h>
#include <ros/ros.h>


//Constructor sets default setpoint type mask
PositionHoldState::PositionHoldState() {
    setpoint_.type_mask = default_mask;
}

//Handles incoming events
void PositionHoldState::handleEvent(ControlFSM& fsm, const EventData& event) {
    isActive_ = true;
    if(event.isValidCMD()) {
        //All valid command needs to go via the GOTO state
        fsm.transitionTo(ControlFSM::GOTOSTATE, this, event);
    } else if(event.isValidRequest()) {
        switch(event.request) {
            case RequestType::GOTO:
                fsm.transitionTo(ControlFSM::GOTOSTATE, this, event);
                break;
            case RequestType::LAND:
                fsm.transitionTo(ControlFSM::LANDSTATE, this, event);
                break;

            /*case RequestType::TRACKGB:
                fsm.transitionTo(ControlFSM::TRACKGBSTATE, this, event);
                break;
            */
            default:
                fsm.handleFSMWarn("Transition not allowed");
                break;
        }
    }
}

void PositionHoldState::stateBegin(ControlFSM& fsm, const EventData& event) {
    if(pFsm_ == nullptr) {
        pFsm_ = &fsm;
    }
    safeHoverAlt_ = FSMConfig::SafeHoverAltitude;
    //No need to check other commands
    if(event.isValidCMD()) {
        //All valid commands need to go to correct place on arena before anything else
        fsm.transitionTo(ControlFSM::GOTOSTATE, this, event);
        return;
    }

    const geometry_msgs::PoseStamped* pose = fsm.getPositionXYZ();
    //GoTo blind hover if position not valid, should never occur
    if(pose == nullptr) {
        if(event.isValidCMD()) {
            event.eventError("No valid position!");
        }
        EventData nEvent;
        nEvent.eventType = EventType::POSLOST;
        fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, nEvent); 
        return;
    }

    //Set setpoint to current position
    setpoint_.position.x = pose->pose.position.x;
    setpoint_.position.y = pose->pose.position.y;
    //Keep old altitude if abort
    //TODO Should we use an default hover altitude in case of ABORT?
    if(!event.isValidRequest() || event.request != RequestType::ABORT) {
        setpoint_.position.z = pose->pose.position.z;
    }

    setpoint_.yaw = (float) fsm.getMavrosCorrectedYaw();
    
}

void PositionHoldState::stateInit(ControlFSM &fsm) {
    pFsm_ = &fsm;
    lidarSub_ = fsm.nodeHandler_.subscribe(FSMConfig::LidarTopic, 1, &PositionHoldState::obsCB, this);
}

bool PositionHoldState::stateIsReady(ControlFSM &fsm) {

    //Return true if obstacle detection is disabled
    if(!FSMConfig::RequireObstacleDetection) return true;

    //Skipping check is allowed in debug mode
    if(!FSMConfig::RequireAllDataStreams) return true;

    if(lidarSub_.getNumPublishers() > 0) {
        return true;
    } else {
        fsm.handleFSMWarn("No lidar publisher in posHold");
        return false;
    }
}

void PositionHoldState::obsCB(const ascend_msgs::PointArray::ConstPtr& msg) {
    //TODO TEST!!!
    //Only check if neccesary
    if(!isActive_ || setpoint_.position.z >= safeHoverAlt_) {
        return;
    }
    if(pFsm_ == nullptr) {
        ROS_ERROR("FSM pointer = nullptr! Critical!");
        return; //Avoids nullpointer exception
    }
    auto points = msg->points;
    const geometry_msgs::PoseStamped* pPose = pFsm_->getPositionXYZ();
    //Should never happen!
    if(pPose == nullptr) {
        //No valid XY position available, no way to determine distance to GB
        pFsm_->handleFSMError("Position not available! Should not happen!");
        return;
    }
    //No need to check obstacles if they're too close
    if(pPose->pose.position.z >= FSMConfig::SafeHoverAltitude) {
        return;
    }

    double droneX = pPose->pose.position.x;
    double droneY = pPose->pose.position.y;
    for(int i = 0; i < points.size(); ++i) {
        double distSquared = std::pow(droneX - points[i].x, 2) + std::pow(droneY - points[i].y, 2);
        if(distSquared < std::pow(FSMConfig::ObstacleTooCloseDist, 2)) {
            setpoint_.position.z = safeHoverAlt_;
            return; //No need to check the rest!
        }
    }


}

void PositionHoldState::stateEnd(ControlFSM &fsm, const EventData& eventData) {
    isActive_ = false;
}

//Returns setpoint
const mavros_msgs::PositionTarget* PositionHoldState::getSetpoint() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

void PositionHoldState::handleManual(ControlFSM &fsm) {
    RequestEvent manualEvent(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUALFLIGHTSTATE, this, manualEvent);
}
