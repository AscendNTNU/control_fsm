#include "control_fsm/LandState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/ControlFSM.hpp"

LandState::LandState() {
    _setpoint.type_mask = default_mask | SETPOINT_TYPE_LAND;
    _setpoint.position.z = -1; //Shouldnt matter
}

void LandState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidRequest()) {
        if(event.request == RequestType::ABORT) {
            if(_cmd.isValidCMD()) {
                _cmd.eventError("ABORT request sent. Aborting command");
                _cmd = EventData();
            }
            fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event);
            return;
        } else {
            fsm.handleFSMWarn("Illegal transition request!");
        }
    } else if(event.eventType == EventType::GROUNDDETECTED) {
        //Land is finished
        if(_cmd.isValidCMD()) {
            //Only landxy should occur!
            if(_cmd.commandType == CommandType::LANDXY) {
                _cmd.finishCMD();
            } else {
                _cmd.eventError("Wrong CMD type!");
                fsm.handleFSMError("Invalid CMD type in land state!");
            }
            _cmd = EventData();
        }
        fsm.transitionTo(ControlFSM::IDLESTATE, this, event);
    } else if(event.isValidCMD()) {
        if(_cmd.isValidCMD()) {
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
        _cmd = event;
        _cmd.sendFeedback("Landing!");
    }
    const geometry_msgs::PoseStamped* pPose = fsm.getPositionXYZ();
    if(pPose != nullptr) {
        _setpoint.position.x = pPose->pose.position.x;
        _setpoint.position.y = pPose->pose.position.y;
        //Set yaw setpoint based on current rotation
        //TODO Make sure this cast works fine
        _setpoint.yaw = (float) fsm.getMavrosCorrectedYaw();
    } else {
        //Should never occur
        RequestEvent abortEvent(RequestType::ABORT);
        if(_cmd.isValidCMD()) {
            _cmd.eventError("No valid position");
            _cmd = EventData();
        }
        fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, abortEvent);
    }
}

void LandState::loopState(ControlFSM& fsm) {
    if(fsm._landDetector.isOnGround()) {
        if(_cmd.isValidCMD()) {
            //Only landxy should occur!
            if(_cmd.commandType == CommandType::LANDXY) {
                _cmd.finishCMD();
            } else {
                _cmd.eventError("Wrong CMD type!");
                fsm.handleFSMError("Invalid CMD type in land state!");
            }
            _cmd = EventData();
        }
        RequestEvent idleRequest(RequestType::IDLE);
        fsm.transitionTo(ControlFSM::IDLESTATE, this, idleRequest);
    }
}

const mavros_msgs::PositionTarget* LandState::getSetpoint() {
    _setpoint.header.stamp = ros::Time::now();
    return &_setpoint;
}