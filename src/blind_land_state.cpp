#include "control_fsm/blind_land_state.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/control_fsm.hpp"

BlindLandState::BlindLandState() {
    setpoint_.type_mask = default_mask | IGNORE_PX | IGNORE_PY | SETPOINT_TYPE_LAND;
}

void BlindLandState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.event_type == EventType::GROUNDDETECTED) {
        //Land completed
        fsm.transitionTo(ControlFSM::IDLE_STATE, this, event);
    } else if(event.isValidCMD()) {
        //Blind land not part of normal operation. 
        //Any command will be ignored!
        event.eventError("CMD rejected!");
        fsm.handleFSMWarn("Blind land not part of normal operation! Ignoring commands!");
    } else if(event.isValidRequest()) {
        if(event.request == RequestType::ABORT) {
            fsm.handleFSMWarn("Can't ABORT blind land!");
        } else {
            fsm.handleFSMWarn("Illegal transition request!");
        }
    } else {
        fsm.handleFSMDebug("Event ignored!");
    }
}

void BlindLandState::stateBegin(ControlFSM& fsm, const EventData& event) {
    //TODO: Fix blindland and remove transition to posHold.
    fsm.handleFSMWarn("Blind land not tested properly! Not available! Going to posHold!");
    RequestEvent req(RequestType::POSHOLD);
    fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, req);
    ///Get shared_ptr to drones pose
    auto pose_p = control::Pose::getSharedPosePtr();
    setpoint_.yaw = static_cast<float>(pose_p->getMavrosCorrectedYaw());
}

const mavros_msgs::PositionTarget* BlindLandState::getSetpoint() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}


void BlindLandState::handleManual(ControlFSM &fsm) {
    RequestEvent req_event(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUAL_FLIGHT_STATE, this, req_event);
}
