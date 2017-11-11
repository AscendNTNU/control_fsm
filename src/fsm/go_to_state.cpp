#include "control/fsm/go_to_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control/fsm/control_fsm.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <geometry_msgs/Point32.h>
#include <control/tools/logger.hpp>
#include <control/exceptions/PoseNotValidException.hpp>
#include "control/tools/config.hpp"
#include "control/tools/target_tools.hpp"

constexpr double PI = 3.14159265359;
constexpr double MAVROS_YAW_CORRECTION_PI_HALF = 3.141592653589793 / 2.0;



GoToState::GoToState() : StateInterface::StateInterface() {
    setpoint_.type_mask = default_mask;
}

void GoToState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidRequest()) {
        if(event.request == RequestType::ABORT) {
            if(cmd_.isValidCMD()) {
                cmd_.eventError("ABORT");
                cmd_ = EventData();
            }
            fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, event);
        } 
        else if(event.request == RequestType::POSHOLD) {
            if(cmd_.isValidCMD()) {
                control::handleWarnMsg("ABORT CMD before sending manual request!");
            }
            else {
                fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, event);
            }
        } 
        else if(event.request == RequestType::GOTO) {
            if(cmd_.isValidCMD()) {
                control::handleWarnMsg("ABORT CMD before sending manual request!");
            } 
            else {
                fsm.transitionTo(ControlFSM::GO_TO_STATE, this, event);
            }
        } 
        else {
            control::handleWarnMsg("Illegal transiton request");
        }
    } 
    else if(event.isValidCMD()) {
        if(cmd_.isValidCMD()) {
            event.eventError("ABORT request should be sent before new command");
        } 
        else {
            fsm.transitionTo(ControlFSM::GO_TO_STATE, this, event); //Transition to itself
        }
    }
}

void GoToState::stateBegin(ControlFSM& fsm, const EventData& event) {
    is_active_ = true;
    cmd_ = event;
    //Has not arrived yet
    delay_transition_.enabled = false;

    if(!event.position_goal.valid) {
        if(cmd_.isValidCMD()) {
            event.eventError("No valid position target");
            cmd_ = EventData();
        }
        RequestEvent abort_event(RequestType::ABORT);
        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
        return;
    }

    ///Get shared_ptr to drones pose
    try {
        auto pose_p = control::Pose::getSharedPosePtr();
        control::Point position = pose_p->getPositionXYZ();
        // Set setpoint
        setpoint_.position.x = cmd_.position_goal.x;
        setpoint_.position.y = cmd_.position_goal.y;
        setpoint_.position.z = cmd_.position_goal.z;
        setpoint_.yaw = static_cast<float>(control::getMavrosCorrectedTargetYaw(pose_p->getYaw()));
    } catch(const std::exception& e) {
        //Transition to position hold if no pose available
        control::handleCriticalMsg(e.what());
        control::handleErrorMsg("No pose available - aborting goto");
        RequestEvent abort_event(RequestType::ABORT);
        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
    }
}

void GoToState::stateEnd(ControlFSM& fsm, const EventData& event) {
    is_active_ = false;
}

void GoToState::loopState(ControlFSM& fsm) {

    //Get position
    std::shared_ptr<control::Pose> pose_p;
    try {
        pose_p = control::Pose::getSharedPosePtr();
        if(!pose_p->isPoseValid()) {
            //Throw exception if pose is invalid
            throw control::PoseNotValidException();
        }
    } catch(const std::exception& e) {
        //Transition back to poshold if exception is thrown
        EventData event;
        event.event_type = EventType::POSLOST;
        if(cmd_.isValidCMD()) {
            cmd_.eventError("No position");
            cmd_ = EventData();
        }

        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, event);
        return;
    }

    //Get position
    control::Point current_position = pose_p->getPositionXYZ();

    //Check if destination is reached!
    double delta_x = current_position.x - cmd_.position_goal.x;
    double delta_y = current_position.y - cmd_.position_goal.y;
    double delta_z = current_position.z - cmd_.position_goal.z;
    bool xy_reached = (std::pow(delta_x, 2) + std::pow(delta_y, 2)) <= std::pow(dest_reached_margin_, 2);
    bool z_reached = (std::fabs(delta_z) <= control::Config::altitude_reached_margin);
    bool yaw_reached = (std::fabs(pose_p->getMavrosCorrectedYaw() - setpoint_.yaw) <= yaw_reached_margin_);
    //If destination is reached, begin transition to another state

    if(xy_reached && z_reached && yaw_reached) {
        destinationReached(fsm);
    }
    else{
        delay_transition_.enabled = false;
    }
}

//Returns valid setpoint
const mavros_msgs::PositionTarget* GoToState::getSetpointPtr() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}



//Initialize state
void GoToState::stateInit(ControlFSM& fsm) {
    using control::Config;
    //TODO Uneccesary variables - Config can be used directly
    //Set state variables
    delay_transition_.delayTime = ros::Duration(Config::go_to_hold_dest_time);
    dest_reached_margin_ = Config::dest_reached_margin;
    setpoint_reached_margin_ = Config::setpoint_reached_margin;
    yaw_reached_margin_ = Config::yaw_reached_margin;

    control::handleInfoMsg("GoTo init completed!");
}

//Calculates a yaw setpoints that is a multiple of 90 degrees
//and is as close to the path direction as possible 
//NOTE - method assumes dx and dy is not equal to zero
double GoToState::calculatePathYaw(double dx, double dy) {
    //Avoid fatal error if dx and dy is too small
    //If method is used correctly this should NEVER be a problem
    if(std::fabs(dx * dx + dy * dy) < 0.001) {
        return 0;
    }
    /*
    angle = acos(([dx, dy] dot [1,0]) / (norm([dx, dy]) * norm([1,0]))) = acos(dx / (norm([dx, dy]) * 1))
    */
    double angle = std::acos(dx / std::sqrt(dx * dx + dy * dy));

    //Select closest multiple of 90 degrees
    if(angle > 3 * PI / 4) {
        angle = PI;
    } 
    else if(angle > PI/4) {
        angle = PI/2.0;
    } 
    else {
        angle = 0;
    }
    //Invert if dy is negative
    if(dy < 0) {
        angle = -angle;
    }

    return angle;
}

bool GoToState::stateIsReady(ControlFSM &fsm) {
    return true;
}

void GoToState::handleManual(ControlFSM &fsm) {
    cmd_.eventError("Lost OFFBOARD");
    cmd_ = EventData();
    RequestEvent manual_event(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUAL_FLIGHT_STATE, this, manual_event);
}


void GoToState::destinationReached(ControlFSM &fsm){
    //Hold current position for a duration - avoiding unwanted velocity before doing anything else
    if(!delay_transition_.enabled) {
        delay_transition_.started = ros::Time::now();
        delay_transition_.enabled = true;

        if(cmd_.isValidCMD()) {
            cmd_.sendFeedback("Destination reached, letting drone slow down before transitioning!");
        }
    }
    //Delay transition
    if(ros::Time::now() - delay_transition_.started < delay_transition_.delayTime) {
        return;
    } 
    //Transition to correct state
    if(cmd_.isValidCMD()) {
        switch(cmd_.command_type) {
            case CommandType::LANDXY:
                fsm.transitionTo(ControlFSM::LAND_STATE, this, cmd_);
                break;
            //TODO(rendellc): why is this commented out?
            /*
            case CommandType::LANDGB:
                fsm.transitionTo(ControlFSM::TRACK_GB_STATE, this, cmd_);
                break;
            */
            case CommandType::GOTOXYZ: {
                cmd_.finishCMD();
                RequestEvent doneEvent(RequestType::POSHOLD);
                fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, doneEvent);
                }
                break;
            default:
                control::handleWarnMsg("Unrecognized command type");
                break;
        }
    } 
    else {
        RequestEvent posHoldEvent(RequestType::POSHOLD);
        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, posHoldEvent);
    }

    delay_transition_.enabled = false;
}
