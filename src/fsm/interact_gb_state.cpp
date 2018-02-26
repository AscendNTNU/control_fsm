#include "control/fsm/interact_gb_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <ros/ros.h>
#include <control/fsm/control_fsm.hpp>
#include <control/tools/logger.hpp>
#include <control/tools/ground_robot_handler.hpp>

//Should this be here?
enum class LOCAL_STATE{IDLE, LAND, RECOVER};
//Just dummys
const double THRESHOLD = 0.5;
const double HEIGHT_THRESHOLD = 3;
//Struct for controlling local state.
//Maybe add a method for doing state validation
struct{
    LOCAL_STATE state;
    bool valid = false;
}_local_state;

void
idleStateHandler(geometry_msgs::PoseStamped& gb_pose, geometry_msgs::PoseStamped& drone_pose, ControlFSM& fsm, InteractGBState* self)
{
    
    double distance_to_gb = sqrt(pow((gb_pose.pose.position.x - drone_pose.pose.position.x),2)
                            + pow((gb_pose.pose.position.y - drone_pose.pose.position.y),2));
    
    //Do checks here and transition to land
    if (distance_to_gb > THRESHOLD || drone_pose.pose.position.y > HEIGHT_THRESHOLD || !control::DroneHandler::isPoseValid())
    {
        _local_state.state = LOCAL_STATE::RECOVER;
        _local_state.valid = false;
    }
    else
    {
        _local_state.state = LOCAL_STATE::LAND;
        _local_state.valid = true;
    }

}

void
landStateHandler(geometry_msgs::PoseStamped& gb_pose, geometry_msgs::PoseStamped& drone_pose, ControlFSM& fsm, InteractGBState* self)
{
    //TODO add Chris' algorithm here.


    if(/*has landed*/true)
    {
        //Success
        _local_state.state = LOCAL_STATE::RECOVER;
        _local_state.valid = true;
    }
}

void
recoverStateHandler(geometry_msgs::PoseStamped& drone_pose, ControlFSM& fsm, InteractGBState* self)
{
    if (!_local_state.valid)
    {
        RequestEvent abort_event(RequestType::ABORT);
        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, self, abort_event);
        _local_state.valid = false;
    }
    if (drone_pose.pose.position.y < HEIGHT_THRESHOLD)
    {

    }
    if (/*Drone has landed*/true)
    {
        //Maybe transition to idle instead and let that take care of pre takeoff checks?
        RequestEvent takeoff_request(RequestType::TAKEOFF);
        fsm.transitionTo(ControlFSM::TAKEOFF_STATE, self, takeoff_request);
    }
    //This should be the natural transition after a successfull land and recover.
    RequestEvent transition_event(RequestType::POSHOLD);
    fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, self, transition_event);
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
    _local_state.state = LOCAL_STATE::IDLE;
    _local_state.valid = true;
}

void InteractGBState::loopState(ControlFSM& fsm) {
    //TODO Run Chris's algorithm
    //TODO Remove thought comments
    auto& drone_pose = control::DroneHandler::getCurrentPose().pose;
    geometry_msgs::PoseStamped& gb_pose{};

    //Checks if we still have visible ground robots to interact with
    //Is the transition to hold pos correct?
    if (/*!gbIsVisible*/false)
    {
        _local_state.state = LOCAL_STATE::RECOVER;
        _local_state.valid = false;
    }

    switch(_local_state.state)
    {
        case LOCAL_STATE::IDLE:
        idleStateHandler(gb_pose, drone_pose, fsm, this)
        break;

        case LOCAL_STATE::LAND:
        landStateHandler(gb_pose, drone_pose,fsm, this)
        break;

        case LOCAL_STATE::RECOVER:
        recoverStateHandler(drone_pose, fsm, this)
        break;

        default:
        // Should never happen.
        _local_state.valid = false;
        break;
    }
    
    if (_local_state.state == LOCAL_STATE::RECOVER && _local_state.valid == true)
    {
        RequestEvent transition(RequestType::POSHOLD);
        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, transition);
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
