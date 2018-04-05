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
        if(event.request == RequestType::PREFLIGHT) {
            control::handleWarnMsg("Going back to preflight, land drone before offboard");
            fsm.transitionTo(ControlFSM::PREFLIGHT_STATE, this, event);
        } else {
            control::handleWarnMsg("Invalid transition request");
        }
    } else if(event.isValidCMD()) {
        event.eventError("CMD rejected!");
        control::handleWarnMsg("Drone is not yet active - commands ignored");
    } else if(event.event_type == EventType::AUTONOMOUS) {
        if(LandDetector::isOnGround()) {
            fsm.transitionTo(ControlFSM::IDLE_STATE, this, event); //Transition to IDLE_STATE
        } else {
            fsm.transitionTo(ControlFSM::BLIND_HOVER_STATE, this, event); //Transition to BLIND_HOVER_STATE
        }
    } else {
        control::handleInfoMsg("Event ignored");
    }
}

void ManualFlightState::loopState(ControlFSM& fsm) {
    try {
        auto pose_stamped = control::DroneHandler::getCurrentLocalPose();
        auto &position = pose_stamped.pose.position;

        if (LandDetector::isOnGround()) {
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
