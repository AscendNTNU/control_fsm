#include "control/fsm/land_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <ros/ros.h>
#include <control/tools/target_tools.hpp>
#include "control/fsm/control_fsm.hpp"

LandState::LandState() {
    //Ignoring PX and PY - lands without XY feedback
    setpoint_.type_mask = default_mask | SETPOINT_TYPE_LAND | IGNORE_PX | IGNORE_PY;
    setpoint_.position.z = -1; //Shouldnt matter
}

void LandState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidRequest()) {
        if(event.request == RequestType::ABORT) {
            if(cmd_.isValidCMD()) {
                cmd_.eventError("ABORT request sent. Aborting command");
                cmd_ = EventData();
            }
            fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, event);
            return;
        } else {
            fsm.handleFSMWarn("Illegal transition request!");
        }
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
    auto pose_p = control::Pose::getSharedPosePtr();
    control::Point current_position = pose_p->getPositionXYZ();
    if(pose_p != nullptr) {
        //Position XY is ignored in typemask, but the values are set as a precaution.
        setpoint_.position.x = current_position.x;
        setpoint_.position.y = current_position.y;
        //Set yaw setpoint based on current rotation
        setpoint_.yaw = control::getMavrosCorrectedTargetYaw(pose_p->getYaw());
    } else {
        //Should never occur
        RequestEvent abort_event(RequestType::ABORT);
        if(cmd_.isValidCMD()) {
            cmd_.eventError("No valid position");
            cmd_ = EventData();
        }
        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
    }
}

void LandState::loopState(ControlFSM& fsm) {
    if(fsm.land_detector_.isOnGround()) {
        if(cmd_.isValidCMD()) {
            //Only landxy should occur!
            if(cmd_.command_type == CommandType::LANDXY) {
                cmd_.finishCMD();
            } else {
                cmd_.eventError("Wrong CMD type!");
                fsm.handleFSMError("Invalid CMD type in land state!");
            }
            cmd_ = EventData();
        }
        RequestEvent idle_request(RequestType::IDLE);
        fsm.transitionTo(ControlFSM::IDLE_STATE, this, idle_request);
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
    RequestEvent manual_event(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUAL_FLIGHT_STATE, this, manual_event);
}
