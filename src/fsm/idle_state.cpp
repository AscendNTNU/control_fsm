#include "control/fsm/idle_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <ros/ros.h>
#include <control/tools/logger.hpp>
#include "control/fsm/event_data.hpp"
#include "control/fsm/control_fsm.hpp"

ros::Publisher pose_pub;
geometry_msgs::PoseStamped idle_pose;

//Sets setpoint type to IDLE
IdleState::IdleState() {
    setpoint_ = mavros_msgs::PositionTarget();
    setpoint_.type_mask = default_mask          |
                          SETPOINT_TYPE_IDLE    |
                          IGNORE_PX             |
                          IGNORE_PY             |
                          IGNORE_PZ             |
                          IGNORE_YAW;
}

void IdleState::handleEvent(ControlFSM& fsm, const EventData& event) {
    //All commands needs to get to position hold first
    if(event.isValidRequest()) {
        if(event.request == RequestType::TAKEOFF) {
            fsm.transitionTo(ControlFSM::TAKEOFF_STATE, this, event);
        } else {
            control::handleWarnMsg("Invalid transition request");
        }
    } else if(event.isValidCMD()) {
        fsm.transitionTo(ControlFSM::TAKEOFF_STATE, this, event);
    } else  {
        control::handleInfoMsg("Event ignored");
    }
}

void IdleState::stateInit(ControlFSM& fsm) {
    using control::Config;
    using geometry_msgs::PoseStamped;
    auto& nh = fsm.node_handler_;
    pose_pub = nh.advertise<PoseStamped>(Config::idle_pose_topic, 1);
}

void IdleState::stateBegin(ControlFSM& fsm, const EventData& event) {
    //Drone assumed to be on ground, stationary (or it will be soon anyways :') )
    idle_pose = control::DroneHandler::getCurrentLocalPose();
    idle_pose.pose.position.z = 0.0;
}

void IdleState::loopState(ControlFSM& fsm) {
    //Publish to pose filter, forcing z = 0 when on ground!
    idle_pose.header.stamp = ros::Time::now();
    pose_pub.publish(idle_pose);
}

const mavros_msgs::PositionTarget* IdleState::getSetpointPtr() {
    //Sets timestamp, and returns setpoint_ as const pointer
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

void IdleState::handleManual(ControlFSM &fsm) {
    RequestEvent manual_event(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUAL_FLIGHT_STATE, this, manual_event);
}



ascend_msgs::ControlFSMState IdleState::getStateMsg() const {
    using ascend_msgs::ControlFSMState;
    ControlFSMState msg;
    msg.name = getStateName();
    msg.state_id = ControlFSMState::IDLE_STATE;
    return msg;
}
