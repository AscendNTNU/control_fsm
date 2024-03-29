#include "control/fsm/blind_hover_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <ros/ros.h>
#include <control/tools/target_tools.hpp>
#include <control/tools/logger.hpp>
#include "control/fsm/control_fsm.hpp"
#include "control/fsm/event_data.hpp"
#include "control/tools/config.hpp"
#include "control/tools/drone_handler.hpp"

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

bool poseIsValid() {
    using control::Config;
    using control::DroneHandler;
    auto pos = DroneHandler::getCurrentLocalPose().pose.position;
    bool valid_altitude = (pos.z >= Config::min_in_air_alt + Config::altitude_reached_margin);

    if(!control::DroneHandler::isLocalPoseValid()) return false;
    return valid_altitude;
}

void BlindHoverState::stateBegin(ControlFSM& fsm, const EventData& event ) {
    //Set default blind hover altitude
    setpoint_.position.z = control::Config::blind_hover_alt;
    try {
        //If full position is valid - no need to blind hover
        if(poseIsValid()) {
            if(event.isValidCMD()) {
                fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, event); //Pass command on to next state
            } else {
                RequestEvent req_event(RequestType::POSHOLD);
                //Set target altitude if passed on from takeoff
                if(event.setpoint_target.z_valid) {
                    req_event.setpoint_target = event.setpoint_target;
                }
                fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, req_event);
            }
        }
        return;
    } catch(const std::exception& e) {
        //Exceptions should NEVER occur! Critical bug!
        control::handleCriticalMsg(e.what());
    }
    //If full position is valid - no need to blind hover
    if(event.isValidCMD()) {
        cmd_ = event;
    }

    // Obstacle avoidance has no data to go by, so disable it
    fsm.obstacle_avoidance_.relaxResponsibility();

    control::handleWarnMsg("No valid pose available, blindhovering!");
}

void BlindHoverState::loopState(ControlFSM& fsm) {
    try {
        //Transition to position hold when position is valid.
        if(poseIsValid()) {
            if (cmd_.isValidCMD()) {
                fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, cmd_);
                cmd_ = EventData(); //Reset cmd_
            } else {
                RequestEvent event(RequestType::POSHOLD);
                fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, event);
            }
        }
    } catch(const std::exception& e) {
        control::handleCriticalMsg(e.what());
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



ascend_msgs::ControlFSMState BlindHoverState::getStateMsg() const {
    using ascend_msgs::ControlFSMState;
    ControlFSMState msg;
    msg.name = getStateName();
    msg.state_id = ControlFSMState::BLIND_HOVER_STATE;
    return msg;
}
