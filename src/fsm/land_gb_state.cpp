#include "control/fsm/land_gb_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <ros/ros.h>
#include <control/fsm/control_fsm.hpp>
#include <control/tools/logger.hpp>
#include <control/tools/ground_robot_handler.hpp>

using GRstate = ascend_msgs::GRState;
using PoseStamped = geometry_msgs::PoseStamped;
using PosTarget = mavros_msgs::PositionTarget;

enum class LOCAL_STATE{IDLE, LAND, RECOVER};
//Just dummies
constexpr double DISTANCE_THRESHOLD = 0.5;
constexpr double HEIGHT_THRESHOLD = 0.5;

//Struct for controlling local state.
struct{
    LOCAL_STATE state;
    bool valid = false;
    bool transition_valid = false;
}local_state;

void
idleStateHandler(const GRstate& gb_pose, const PoseStamped& drone_pose) {
    auto& drone_pos = drone_pose.pose.position;

    double distance_to_gb = sqrt(pow((gb_pose.x - drone_pos.x),2)
                                + pow((gb_pose.y - drone_pos.y),2));
    
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
landStateHandler(const GRstate& gb_pose, const PoseStamped& drone_pose, const PosTarget* setpoint)
{
    //TODO add Chris' algorithm here.
    if(LandDetector::isOnGround())
    {
        //Success
        local_state.state = LOCAL_STATE::RECOVER;
        local_state.valid = true;
    }
    else if(false)
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

    //Sets the local state to idle, awaiting clearance for landing.
    local_state.state = LOCAL_STATE::IDLE;
    local_state.valid = true;
}

void LandGBState::loopState(ControlFSM& fsm) {
    //TODO Run Chris's algorithm
    //TODO Remove thought comments
    auto& drone_pose = control::DroneHandler::getCurrentPose();
    ascend_msgs::GRState gb_pose = {};
    auto* setpoint = this->getSetpointPtr();

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
        landStateHandler(gb_pose, drone_pose, setpoint);
        break;

        case LOCAL_STATE::RECOVER:
        recoverStateHandler(drone_pose);
        break;

        default:
        local_state.valid = false;
        control::handleErrorMsg("Local interact gb state failure!");
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


ascend_msgs::ControlFSMState LandGBState::getStateMsg() {
    using ascend_msgs::ControlFSMState;
    ControlFSMState msg;
    msg.name = getStateName();
    msg.state_id = ControlFSMState::INTERACT_GB_STATE;
    return msg;
}
