#include "control/fsm/land_gb_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <ros/ros.h>
#include <control/fsm/control_fsm.hpp>
#include <control/tools/logger.hpp>
#include <control/tools/ground_robot_handler.hpp>

#include <functional>

using GRstate = ascend_msgs::GRState;
using PoseStamped = geometry_msgs::PoseStamped;
using PosTarget = mavros_msgs::PositionTarget;

enum class LocalState: int {IDLE, LAND, RECOVER, INVALID, COMPLETED};
//Parameters for thresholds, do they deserve a spot in the config?
constexpr double DISTANCE_THRESHOLD = 0.5;
constexpr double HEIGHT_THRESHOLD = 0.5;

//Forward declarations
LocalState idleStateHandler(const GRstate& gb_pose, const PoseStamped& drone_pose, const PosTarget* setpoint);
LocalState landStateHandler(const GRstate& gb_pose, const PoseStamped& drone_pose, const PosTarget* setpoint);
LocalState recoverStateHandler(const GRstate& gb_state, const PoseStamped& drone_pose, const PosTarget* setpoint);
LocalState invalidStateHandler(const GRstate& gb_state, const PoseStamped& drone_pose, const PosTarget* setpoint);
LocalState completedStateHandler(const GRstate& gb_state, const PoseStamped& drone_pose, const PosTarget* setpoint);

//State function array, containing all the state functions.
std::array<std::function<LocalState(const GRstate&, const PoseStamped&, const PosTarget*)>, 5> state_function_array = {
    idleStateHandler, landStateHandler, recoverStateHandler, invalidStateHandler, completedStateHandler };

//Holds the current state function to be run every loop
std::function<LocalState(const GRstate&, const PoseStamped&, const PosTarget*)> stateFunction = idleStateHandler;
//Holds the current state
LocalState local_state = LocalState::IDLE;

LocalState idleStateHandler(const GRstate& gb_pose, const PoseStamped& drone_pose, const PosTarget* dummy) {
    auto& drone_pos = drone_pose.pose.position;
    double distance_to_gb = sqrt(pow((gb_pose.x - drone_pos.x),2)
                                + pow((gb_pose.y - drone_pos.y),2));
    //Do checks here and transition to land
    if (!gb_pose.downward_tracked){
        return LocalState::RECOVER;
    }
    if (distance_to_gb < DISTANCE_THRESHOLD || drone_pose.pose.position.y < HEIGHT_THRESHOLD) {
        return LocalState::LAND;
    }
    else {
        return LocalState::RECOVER;
    }
}

LocalState landStateHandler(const GRstate& gb_pose, const PoseStamped& drone_pose, const PosTarget* setpoint) {
    //TODO add Chris' algorithm here.
    auto* land_detector = LandDetector::getSharedInstancePtr();
    if(land_detector->isOnGround()) {
        //Success
        return LocalState::RECOVER;
    }
    else if(false) {
        return LocalState::INVALID;
    }
}

LocalState recoverStateHandler(const GRstate& gb_pose, const PoseStamped& drone_pose, const PosTarget* setpoint) {
    if (drone_pose.pose.position.z < HEIGHT_THRESHOLD) {
        // Setpoint to a higher altitude?

        if (/**/false) {
            
        }
        return LocalState::COMPLETED;
    }
}

LocalState invalidStateHandler(const GRstate& gb_state, const PoseStamped& drone_pose, const PosTarget* setpoint) {

    return LocalState::INVALID;
}

LocalState completedStateHandler(const GRstate& gb_pose, const PoseStamped& drone_pose, const PosTarget* setpoint) {
    return LocalState::COMPLETED;
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
        stateFunction = idleStateHandler;
    } catch(const std::exception& e) {
        //Critical bug - no recovery
        //Transition to position hold if no pose available
        control::handleCriticalMsg(e.what());
        RequestEvent abort_event(RequestType::ABORT);
        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
    }
}

void LandGBState::loopState(ControlFSM& fsm) {
    //TODO Run Chris's algorithm

    auto& drone_pose = control::DroneHandler::getCurrentPose();
    ascend_msgs::GRState gb_pose = {};
    auto* setpoint = this->getSetpointPtr();

    //Checks if we still have visible ground robots to interact with
    if (!gb_pose.downward_tracked) {
        local_state = LocalState::RECOVER;
        stateFunction = state_function_array[static_cast<int>(LocalState::RECOVER)];
    }

    // Loops the current and sets the next state.
    local_state = stateFunction(gb_pose, drone_pose, setpoint);

    // Sets the new state function
    stateFunction = state_function_array[static_cast<int>(local_state)];

    if (local_state == LocalState::RECOVER) {
        if (local_state != LocalState::INVALID) {
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
