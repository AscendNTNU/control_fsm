#include "control/fsm/manual_flight_state.hpp"
#include "control/fsm/control_fsm.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <geometry_msgs/PoseStamped.h>
#include <control/tools/logger.hpp>
#include "control/tools/drone_handler.hpp"

ManualFlightState::ManualFlightState() {
    setpoint_.type_mask = default_mask;
}
//Only check for an abort event
void ManualFlightState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidRequest()) {
        if (event.request == RequestType::BLINDHOVER) {
            next_state_ = RequestType::BLINDHOVER;
        } else if (event.request == RequestType::IDLE) {
            next_state_ = RequestType::IDLE;
        } else {
            control::handleWarnMsg("Invalid transition request");
        }
    } else if(event.isValidCMD()) {
        event.eventError("CMD rejected!");
        control::handleWarnMsg("Drone is not yet active - commands ignored");
    } else if(event.event_type == EventType::AUTONOMOUS) {
        if (next_state_ == RequestType::BLINDHOVER){
            fsm.transitionTo(ControlFSM::BLIND_HOVER_STATE, this, event); 
        } else if (next_state_ == RequestType::IDLE){
            fsm.transitionTo(ControlFSM::IDLE_STATE, this, event); 
        } else {
            control::handleInfoMsg("Unexpected transition request ignored");
        }
    } else {
        control::handleInfoMsg("Event ignored");
    }
}

void printNextState(RequestType next_state) {
    if (next_state == RequestType::BLINDHOVER){
        control::handleInfoMsg("Transition to BLINDHOVER when armed and offboard");
    } else if (next_state == RequestType::IDLE){
        control::handleInfoMsg("Transition to IDLE when armed and offboard");
    }
}

void printNextStateOnChange(RequestType next_state) {
    static RequestType last_printed_state = next_state;
    if (last_printed_state != next_state) {
        printNextState(next_state);
    }

    last_printed_state = next_state;
}

void ManualFlightState::stateBegin(ControlFSM& fsm, const EventData& event) {
    printNextState(next_state_);
}

void ManualFlightState::loopState(ControlFSM& fsm) {
    try {
        const auto pose_stamped = control::DroneHandler::getCurrentLocalPose();
        const auto &position = pose_stamped.pose.position;
        const bool on_ground = LandDetector::isOnGround(); 
        printNextStateOnChange(next_state_);

        if (on_ground) {
            setpoint_.type_mask = default_mask | SETPOINT_TYPE_IDLE; //Send IDLE setpoints while drone is on ground
        } else {
            setpoint_.type_mask = default_mask;
            setpoint_.position.x = position.x;
            setpoint_.position.y = position.y;
            setpoint_.position.z = position.z;
            setpoint_.yaw = static_cast<float>(control::pose::quat2mavrosyaw(pose_stamped.pose.orientation));
        }
    } catch(const std::exception& e) {
        //Exceptions shouldn't occur!
        control::handleWarnMsg(e.what());
    }
}


//Returns setpoint
const mavros_msgs::PositionTarget* ManualFlightState::getSetpointPtr() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

void ManualFlightState::handleManual(ControlFSM &fsm) {
    //Already in manual, nothing to do
}



ascend_msgs::ControlFSMState ManualFlightState::getStateMsg() const {
    using ascend_msgs::ControlFSMState;
    ControlFSMState msg;
    msg.name = getStateName();
    msg.state_id = ControlFSMState::MANUAL_FLIGHT_STATE;
    return msg;
}
