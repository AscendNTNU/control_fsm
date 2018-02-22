#include "control/fsm/interact_gb_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <ros/ros.h>
#include <control/fsm/control_fsm.hpp>
#include <control/tools/logger.hpp>
#include <control/tools/ground_robot_handler.hpp>

enum class LOCAL_STATE{IDLE, LAND, RECOVER};
//Struct for controlling local state.
struct{
    LOCAL_STATE state;
    bool valid = false;
}_local_state;

bool
idleStateHandler()
{
    //Do checks here and transition to land

    if (/*Success*/true)
    {
        _local_state.state = LOCAL_STATE::LAND;
        return true;
    }
}

bool
landStateHandler()
{
    if (/*!checks*/ false)
    {
        //Failure
        _local_state.state = LOCAL_STATE::RECOVER;
        return false;
    }

    if(/*has landed*/true)
    {
        //Success
        _local_state.state = LOCAL_STATE::RECOVER;
        return true;
    }
}

bool
recoverStateHandler()
{
    if (/*!checks*/ false)
    {

        return false;
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

    //Sets the local state to tracking, awaiting clearance for landing.
    _local_state.state = LOCAL_STATE::TRACK;
    _local_state.valid = true;
}

void InteractGBState::loopState(ControlFSM& fsm) {
    //TODO Run Chris's algorithm
    //TODO Remove thought comments

    //Checks if we still have visible ground robots to interact with
    //Is the transition to hold pos correct?
    if (/*!gbIsVisible*/false ||
        (!_local_state.valid && _local_state.state == LOCAL_STATE::RECOVER))
    {
        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE,this, cmd_);
        _local_state.valid = false;
    }

    //Unsure if this way of handling states is efficient.
    switch(_local_state.state)
    {
        case LOCAL_STATE::IDLE:
        _local_state.valid = idleStateHandler()
        break;

        case LOCAL_STATE::LAND:
        //Chris' landing algorithm?
        _local_state.valid = landStateHandler()
        break;

        case LOCAL_STATE::RECOVER:
        _local_state.valid = recoverStateHandler()
        break;

        default:
        _local_state.valid = false;
        break;
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
