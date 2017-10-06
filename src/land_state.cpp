#include "control_fsm/land_state.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/control_fsm.hpp"

LandState::LandState() {
    setpoint_.type_mask = default_mask | SETPOINT_TYPE_LAND;
    setpoint_.position.z = -1; //Shouldnt matter
}

void LandState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidRequest()) {
        if(event.request == RequestType::ABORT) {
            if(cmd_.isValidCMD()) {
                cmd_.eventError("ABORT request sent. Aborting command");
                cmd_ = EventData();
            }
            fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event);
            return;
        } else {
            fsm.handleFSMWarn("Illegal transition request!");
        }
    } else if(event.eventType == EventType::GROUNDDETECTED) {
        //Land is finished
        if(cmd_.isValidCMD()) {
            //Only landxy should occur!
            if(cmd_.commandType == CommandType::LANDXY) {
                cmd_.finishCMD();
            } else {
                cmd_.eventError("Wrong CMD type!");
                fsm.handleFSMError("Invalid CMD type in land state!");
            }
            cmd_ = EventData();
        }
        fsm.transitionTo(ControlFSM::IDLESTATE, this, event);
    } else if(event.isValidCMD()) {
        if(cmd_.isValidCMD()) {
            fsm.handleFSMWarn("ABORT should be sent before new command!");
            event.eventError("ABORT should be sent before new command!");
        } else {
            fsm.handleFSMWarn("Not accepting CMDs before land is completed!");
            event.eventError("Not accpeting CMDs before land is completed!");
        }
    }
}

void LandState::stateBegin(ControlFSM& fsm, const EventData& event) {
    if(event.isValidCMD()) {
        cmd_ = event;
        cmd_.sendFeedback("Landing!");
    }
    const geometry_msgs::PoseStamped* pPose = fsm.getPositionXYZ();
    if(pPose != nullptr) {
        setpoint_.position.x = pPose->pose.position.x;
        setpoint_.position.y = pPose->pose.position.y;
        //Set yaw setpoint based on current rotation
        setpoint_.yaw = (float) fsm.getMavrosCorrectedYaw();
    } else {
        //Should never occur
        RequestEvent abortEvent(RequestType::ABORT);
        if(cmd_.isValidCMD()) {
            cmd_.eventError("No valid position");
            cmd_ = EventData();
        }
        fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, abortEvent);
    }
}

void LandState::loopState(ControlFSM& fsm) {
    if(fsm.landDetector_.isOnGround()) {
        if(cmd_.isValidCMD()) {
            //Only landxy should occur!
            if(cmd_.commandType == CommandType::LANDXY) {
                cmd_.finishCMD();
            } else {
                cmd_.eventError("Wrong CMD type!");
                fsm.handleFSMError("Invalid CMD type in land state!");
            }
            cmd_ = EventData();
        }
        RequestEvent idleRequest(RequestType::IDLE);
        fsm.transitionTo(ControlFSM::IDLESTATE, this, idleRequest);
    }
}

const mavros_msgs::PositionTarget* LandState::getSetpoint() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

void LandState::handleManual(ControlFSM &fsm) {
    if(cmd_.isValidCMD()) {
        cmd_.eventError("Lost OFFBOARD!");
        cmd_ = EventData();
    }
    RequestEvent manualEvent(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUALFLIGHTSTATE, this, manualEvent);
}
