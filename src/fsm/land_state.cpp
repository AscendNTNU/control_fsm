#include "control/fsm/land_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <ros/ros.h>
#include <control/tools/target_tools.hpp>
#include <control/tools/logger.hpp>
#include <control/exceptions/pose_not_valid_exception.hpp>
#include <control/tools/config.hpp>
#include "control/fsm/control_fsm.hpp"
#include "control/tools/drone_handler.hpp"

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
    try {
        if(!control::DroneHandler::isLocalPoseValid()) {
            throw control::PoseNotValidException();
        }
        auto pose_stamped = control::DroneHandler::getCurrentLocalPose();
        auto& position = pose_stamped.pose.position;
        //Position XY is ignored in typemask, but the values are set as a precaution.
        setpoint_.position.x = position.x;
        setpoint_.position.y = position.y;

        //If it is a valid landxy command
        if(cmd_.isValidCMD() && cmd_.command_type == CommandType::LANDXY) {
            //Set xy setpoint to positionGoal if we're close enough for it to be safe
            double xy_dist_square = std::pow(position.x - event.position_goal.x, 2) + std::pow(position.y - event.position_goal.y, 2);
            if(xy_dist_square <= std::pow(control::Config::setpoint_reached_margin, 2)) {
                setpoint_.position.x = event.position_goal.x;
                setpoint_.position.y = event.position_goal.y;
            }
        }

        //Only land blind when the drone is below a certain altitude
        if(position.z >= control::Config::min_in_air_alt) {
            setpoint_.type_mask = default_mask | SETPOINT_TYPE_LAND;
        } else {
            setpoint_.type_mask = default_mask | SETPOINT_TYPE_LAND | IGNORE_PX | IGNORE_PY;
        }

        //Set yaw setpoint based on current rotation
        using control::getMavrosCorrectedTargetYaw;
        using control::pose::quat2yaw;
        auto& quat = pose_stamped.pose.orientation;
        setpoint_.yaw = static_cast<float>(getMavrosCorrectedTargetYaw(quat2yaw(quat)));
    } catch(const std::exception& e) {
        //Exceptions shouldn't occur!
        control::handleCriticalMsg(e.what());
        //Go back to poshold
        RequestEvent abort_event(RequestType::ABORT);
        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
    }
}

void LandState::loopState(ControlFSM& fsm) {
    try {
        auto pose_stamped = control::DroneHandler::getCurrentLocalPose();
        auto& position = pose_stamped.pose.position;
        //Switch to blind land when altitude is below certain limit.
        if(position.z >= control::Config::min_in_air_alt) {
            setpoint_.type_mask = default_mask | SETPOINT_TYPE_LAND;
        } else {
            setpoint_.type_mask = default_mask | SETPOINT_TYPE_LAND | IGNORE_PX | IGNORE_PY;
        }
        //Check landing
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
        control::handleCriticalMsg(e.what());
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



ascend_msgs::ControlFSMState LandState::getStateMsg() {
    using ascend_msgs::ControlFSMState;
    ControlFSMState msg;
    msg.name = getStateName();
    msg.state_id = ControlFSMState::LAND_STATE;
    return msg;
}
