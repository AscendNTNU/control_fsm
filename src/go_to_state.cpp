#include "control_fsm/go_to_state.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/control_fsm.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <geometry_msgs/Point32.h>
#include "control_fsm/fsm_config.hpp"

constexpr double PI = 3.14159265359;
constexpr double PI_HALF = 1.57079632679;



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
                fsm.handleFSMWarn("ABORT CMD before sending manual request!");
            } 
            else {
                fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, event);
            }
        } 
        else if(event.request == RequestType::GOTO) {
            if(cmd_.isValidCMD()) {
                fsm.handleFSMWarn("ABORT CMD before sending manual request!");
            } 
            else {
                fsm.transitionTo(ControlFSM::GO_TO_STATE, this, event);
            }
        } 
        else {
            fsm.handleFSMWarn("Illegal transiton request");
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
    auto pose_p = control::Pose::getSharedPosePtr();
    control::Point position = pose_p->getPositionXYZ();

    // Set setpoint
    setpoint_.position.x = cmd_.position_goal.x;
    setpoint_.position.y = cmd_.position_goal.y;
    setpoint_.position.z = cmd_.position_goal.z;
    setpoint_.yaw = control::getMavrosCorrectedTargetYaw(pose_p->getYaw());
}

void GoToState::stateEnd(ControlFSM& fsm, const EventData& event) {
    is_active_ = false;
}

void GoToState::loopState(ControlFSM& fsm) {

    //Get position
    auto pose_p = control::Pose::getSharedPosePtr();
    control::Point current_position = pose_p->getPositionXYZ();


    //Check that position data is valid
    if(!pose_p->isPoseValid()) {
        EventData event;
        event.event_type = EventType::POSLOST;
        if(cmd_.isValidCMD()) {
            cmd_.eventError("No position");
            cmd_ = EventData();
        }
        
        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, event);
        return;
    }

    //Check if destination is reached!
    double delta_x = current_position.x - cmd_.position_goal.x;
    double delta_y = current_position.y - cmd_.position_goal.y;
    double delta_z = current_position.z - cmd_.position_goal.z;
    bool xy_reached = (std::pow(delta_x, 2) + std::pow(delta_y, 2)) <= std::pow(dest_reached_margin_, 2);
    bool z_reached = (std::fabs(delta_z) <= FSMConfig::altitude_reached_margin);
    bool yaw_reached = (std::fabs(pose_p->getMavrosCorrectedYaw() - setpoint_.yaw) <= yaw_reached_margin_);
    //If destination is reached, begin transition to another state

    if(xy_reached && z_reached && yaw_reached) {
        destinationReached(fsm);
        //Destination reached, no need to excecute the rest of the function
        delay_transition_.enabled = false;
        return;
    }
        
    delay_transition_.enabled = false;
}

//Returns valid setpoint
const mavros_msgs::PositionTarget* GoToState::getSetpoint() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}



//Initialize state
void GoToState::stateInit(ControlFSM& fsm) {
    //TODO Uneccesary variables - FSMConfig can be used directly
    //Set state variables
    delay_transition_.delayTime = ros::Duration(FSMConfig::go_to_hold_dest_time);
    dest_reached_margin_ = FSMConfig::dest_reached_margin;
    setpoint_reached_margin_ = FSMConfig::setpoint_reached_margin;
    yaw_reached_margin_ = FSMConfig::yaw_reached_margin;

    fsm.handleFSMInfo("GoTo init completed!");
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
            case CommandType::GOTOXYZ:
                cmd_.finishCMD();
                RequestEvent doneEvent(RequestType::POSHOLD);
                fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, doneEvent);
                break;
        }
    } 
    else {
        RequestEvent posHoldEvent(RequestType::POSHOLD);
        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, posHoldEvent);
    }
}
