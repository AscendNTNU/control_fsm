#include "control/fsm/land_gb_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <ros/ros.h>
#include <control/fsm/control_fsm.hpp>
#include <control/tools/logger.hpp>
#include <control/tools/ground_robot_handler.hpp>

LandGBState::LandGBState() {
    setpoint_.type_mask = default_mask;
}

void LandGBState::handleEvent(ControlFSM& fsm, const EventData& event) {
    //TODO Handle all transition requests
    if(event.isValidRequest()) {
        if(event.request == RequestType::ABORT) {
            //TODO Blindhover or position hold?
            if(cmd_.isValidCMD()) {
                cmd_.abort();    
            }
            fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, event);
        } else if(event.request == RequestType::POSHOLD) {
            if(cmd_.isValidCMD()) {
                control::handleWarnMsg("CMD still active, abort first!");
            } else {
                fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, event);
            }
        }
    } else if(event.isValidCMD()) {
        if(cmd_.isValidCMD()) {
            control::handleWarnMsg("CMD still active, abort first!");
        } else {
            if(event.command_type == CommandType::TAKEOFF) {
                //Drone already in air, nothing else to do!
                event.finishCMD(); 
            } else {
                //Transition to poshold to execute new cmd
                fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, event);
            }
        }
    }
}

void LandGBState::stateBegin(ControlFSM& fsm, const EventData& event) {
    using control::DroneHandler;
    using control::GroundRobotHandler;
    //Store event
    cmd_ = event;

    try {
        auto& position = DroneHandler::getCurrentPose().pose.position;
        setpoint_.type_mask = default_mask;
        setpoint_.position.x = position.x;
        setpoint_.position.y = position.y;
        setpoint_.position.z = position.z;

        const auto& gb_state = GroundRobotHandler::getCurrentGroundRobots();
        //Check gb_id
        if(cmd_.gb_id < 0 || static_cast<size_t>(cmd_.gb_id) >= gb_state.size()) {
            if(cmd_.isValidCMD()) {
                cmd_.eventError("Invalid GB id!");
                cmd_.clear();
                RequestEvent abort_event(RequestType::ABORT);
                fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
                return;
            }
        }
    } catch(const std::exception& e) {
        //Critical bug - no recovery
        //Transition to position hold if no pose available
        control::handleCriticalMsg(e.what());
        RequestEvent abort_event(RequestType::ABORT);
        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
    }
}

void LandGBState::loopState(ControlFSM& fsm) {
    //TODO Run Chris's algorithm
}

const mavros_msgs::PositionTarget* LandGBState::getSetpointPtr() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

void LandGBState::handleManual(ControlFSM &fsm) {
    if(cmd_.isValidCMD()) {
        cmd_.eventError("OFFBOARD lost");
        cmd_ = EventData();
    }
    RequestEvent manual_event(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUAL_FLIGHT_STATE, this, manual_event);
}
