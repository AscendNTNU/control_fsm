#include "control_fsm/manual_flight_state.hpp"
#include "control_fsm/control_fsm.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <geometry_msgs/PoseStamped.h>

ManualFlightState::ManualFlightState() {
    setpoint_.type_mask = default_mask;
}
//Only check for an abort event
void ManualFlightState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidCMD()) {
        event.eventError("CMD rejected!");
        fsm.handleFSMWarn("Drone is not yet active - commands ignored");
    } else if(event.isValidRequest()) {
        if(event.request == RequestType::PREFLIGHT) {
            fsm.handleFSMWarn("Going back to preflight, land drone before offboard");
            fsm.transitionTo(ControlFSM::PREFLIGHTSTATE, this, event);
        } else {
            fsm.handleFSMWarn("Invalid transition request");
        }
    } else if(event.event_type == EventType::AUTONOMOUS) {
        if(fsm.land_detector_.isOnGround()) {
            fsm.transitionTo(ControlFSM::IDLESTATE, this, event); //Transition to IDLESTATE
        } else {
            fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, event); //Transition to BLINDHOVERSTATE
        }
    } else {
        fsm.handleFSMInfo("Event ignored");
    }
}

void ManualFlightState::loopState(ControlFSM& fsm) {
    const geometry_msgs::PoseStamped* pPose = fsm.getPositionXYZ();
    if(pPose == nullptr) {
        //Should never occur
        fsm.handleFSMError("Position not valid!!");
        setpoint_.type_mask = default_mask | IGNORE_PX | IGNORE_PY | IGNORE_PZ | IGNORE_YAW;
    } else if(fsm.land_detector_.isOnGround()) {
        setpoint_.type_mask = default_mask | SETPOINT_TYPE_IDLE; //Send IDLE setpoints while drone is on ground
    } else {
        setpoint_.type_mask = default_mask;
        setpoint_.position.x = pPose->pose.position.x;
        setpoint_.position.y = pPose->pose.position.y;
        setpoint_.position.z = pPose->pose.position.z;
        setpoint_.yaw = fsm.getMavrosCorrectedYaw();
    }
}


//Returns setpoint
const mavros_msgs::PositionTarget* ManualFlightState::getSetpoint() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

void ManualFlightState::handleManual(ControlFSM &fsm) {
    //Already in manual, nothing to do
}
