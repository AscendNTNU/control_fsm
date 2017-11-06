#include "control/fsm/land_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <ros/ros.h>
#include <control/tools/target_tools.hpp>
#include <control/tools/logger.hpp>
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
            control::handleWarnMsg("Illegal transition request!");
        }
    } else if(event.isValidCMD()) {
        if(cmd_.isValidCMD()) {
            control::handleWarnMsg("ABORT should be sent before new command!");
            event.eventError("ABORT should be sent before new command!");
        } else {
            control::handleWarnMsg("Not accepting CMDs before land is completed!");
            event.eventError("Not accpeting CMDs before land is completed!");
        }
    }
}

void LandState::stateBegin(ControlFSM& fsm, const EventData& event) {
    if(event.isValidCMD()) {
        cmd_ = event;
        cmd_.sendFeedback("Landing!");
    }
}

void LandState::loopState(ControlFSM& fsm) {
    try {
        auto land_detector_p = LandDetector::getSharedInstancePtr();
        if(land_detector_p->isOnGround()) {
            if(cmd_.isValidCMD()) {
                //Only landxy should occur!
                if(cmd_.command_type == CommandType::LANDXY) {
                    cmd_.finishCMD();
                } else {
                    cmd_.eventError("Wrong CMD type!");
                    control::handleErrorMsg("Invalid CMD type in land state!");
                }
                cmd_ = EventData();
            }
            RequestEvent idle_request(RequestType::IDLE);
            fsm.transitionTo(ControlFSM::IDLE_STATE, this, idle_request);
        }
    } catch (const std::exception& e) {
        control::handleErrorMsg(std::string("No land detector available: ") + e.what());
    }
}

const mavros_msgs::PositionTarget* LandState::getSetpointPtr() {
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
