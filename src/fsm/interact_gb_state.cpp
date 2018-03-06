#include "control/fsm/interact_gb_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <ros/ros.h>
#include <control/fsm/control_fsm.hpp>
#include <control/tools/logger.hpp>
#include <control/tools/ground_robot_handler.hpp>

typedef ascend_msgs::GRState GRstate;
typedef geometry_msgs::PoseStamped PoseStamped;
typedef mavros_msgs::PositionTarget PosTarget;

enum class LOCAL_STATE{IDLE, LAND, RECOVER};
//Just dummies
const double DISTANCE_THRESHOLD = 0.5;
const double HEIGHT_THRESHOLD = 0.5;

//Struct for controlling local state.
struct{
    LOCAL_STATE state;
    bool valid = false;
    bool transition_valid = false;
}local_state;

void
idleStateHandler(const GRstate& gb_pose, const PoseStamped& drone_pose) {
    auto& drone_pos = drone_pose.pose.position;

    double distance_to_gb = sqrt(pow((gb_pos.x - drone_pos.x),2)
                                + pow((gb_pos.y - drone_pos.y),2));
    
    //Do checks here and transition to land
    if (distance_to_gb > DISTANCE_THRESHOLD || drone_pose.pose.position.y > HEIGHT_THRESHOLD)
    {
        local_state.state = LOCAL_STATE::RECOVER;
        local_state.valid = false;
    }
    else
    {
        local_state.state = LOCAL_STATE::LAND;
        local_state.valid = true;
    }
}

void
landStateHandler(const GRstate& gb_pose, const PoseStamped& drone_pose, PosTarget* setpoint)
{
    //TODO add Chris' algorithm here.
    if(/*has landed*/true)
    {
        //Success
        local_state.state = LOCAL_STATE::RECOVER;
        local_state.valid = true;
    }
    else if(/*Failed*/false)
    {
        local_state.state = LOCAL_STATE::RECOVER;
        local_state.valid = false;
    }
}

void
recoverStateHandler(const PoseStamped& drone_pose)
{
    if (drone_pose.pose.position.z < HEIGHT_THRESHOLD)
    {
        // Setpoint to a higher altitude?
        if (/**/false)
        {
            
        }
        else
        {
            local_state.valid = true;
            local_state.transition_valid = true;
        }
    }
}

InteractGBState::InteractGBState() {
    setpoint_.type_mask = default_mask;
}

void InteractGBState::handleEvent(ControlFSM& fsm, const EventData& event) {
    //TODO Handle all transition requests
}

void InteractGBState::stateBegin(ControlFSM& fsm, const EventData& event) {
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
                cmd_ = EventData();
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

    //Sets the local state to idle, awaiting clearance for landing.
    local_state.state = LOCAL_STATE::IDLE;
    local_state.valid = true;
}

void InteractGBState::loopState(ControlFSM& fsm) {
    //TODO Run Chris's algorithm
    //TODO Remove thought comments
    auto& drone_pose = control::DroneHandler::getCurrentPose();
    ascend_msgs::GRState gb_pose = {};

    //Checks if we still have visible ground robots to interact with
    if (gb_pose.downward_tracked)
    {
        local_state.state = LOCAL_STATE::RECOVER;
        local_state.valid = false;
    }

    switch(local_state.state)
    {
        case LOCAL_STATE::IDLE:
        idleStateHandler(gb_pose, drone_pose);
        break;

        case LOCAL_STATE::LAND:
        landStateHandler(gb_pose, drone_pose);
        break;

        case LOCAL_STATE::RECOVER:
        recoverStateHandler(drone_pose);
        break;

        default:
        local_state.valid = false;
        control::handleErrorMsg("Local interact gb state failure!")
        break;
    }
    
    if (local_state.state == LOCAL_STATE::RECOVER && local_state.transition_valid)
    {
        if (local_state.valid)
        {
            cmd_.finishCMD();
            RequestEvent transition(RequestType::POSHOLD);
            fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, transition);
        }
        else{
            RequestEvent abort_event(RequestType::ABORT);
            fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
        }
    }

}

const mavros_msgs::PositionTarget* InteractGBState::getSetpointPtr() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

void InteractGBState::handleManual(ControlFSM &fsm) {
    if(cmd_.isValidCMD()) {
        cmd_.eventError("OFFBOARD lost");
        cmd_ = EventData();
    }
    RequestEvent manual_event(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUAL_FLIGHT_STATE, this, manual_event);
}


ascend_msgs::ControlFSMState InteractGBState::getStateMsg() {
    using ascend_msgs::ControlFSMState;
    ControlFSMState msg;
    msg.name = getStateName();
    msg.state_id = ControlFSMState::INTERACT_GB_STATE;
    return msg;
}
