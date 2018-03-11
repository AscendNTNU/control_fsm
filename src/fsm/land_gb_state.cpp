#include "control/fsm/land_gb_state.hpp"
#include "control/tools/setpoint_msg_defines.h"
#include <ros/ros.h>
#include <control/fsm/control_fsm.hpp>
#include <control/tools/logger.hpp>
#include <control/tools/ground_robot_handler.hpp>
#include <control/tools/intercept_groundbot.hpp>

#include <functional>

using GRstate = ascend_msgs::GRState;
using PoseStamped = geometry_msgs::PoseStamped;
using PosTarget = mavros_msgs::PositionTarget;

enum class LocalState: int {IDLE, LAND, RECOVER, ABORT, COMPLETED};
//Parameters for thresholds, do they deserve a spot in the config?
constexpr double DISTANCE_THRESHOLD = 0.5;
constexpr double HEIGHT_THRESHOLD = 0.5;

//Forward declarations
LocalState idleStateHandler(const GRstate& gb_pose, const PoseStamped& drone_pose, PosTarget* setpoint_p);
LocalState landStateHandler(const GRstate& gb_pose, const PoseStamped& drone_pose, PosTarget* setpoint_p);
LocalState recoverStateHandler(const GRstate& gb_state, const PoseStamped& drone_pose, PosTarget* setpoint_p);
LocalState abortStateHandler(const GRstate& gb_state, const PoseStamped& drone_pose, PosTarget* setpoint_p);
LocalState completedStateHandler(const GRstate& gb_state, const PoseStamped& drone_pose, PosTarget* setpoint_p);

//State function array, containing all the state functions.
std::array<std::function<LocalState(const GRstate&, const PoseStamped&, PosTarget*)>, 5> state_function_array = {
    idleStateHandler, landStateHandler, recoverStateHandler, abortStateHandler, completedStateHandler };

//Holds the current state function to be run every loop
std::function<LocalState(const GRstate&, const PoseStamped&, PosTarget*)> stateFunction = idleStateHandler;
//Holds the current state
LocalState local_state = LocalState::IDLE;

LocalState idleStateHandler(const GRstate& gb_pose, const PoseStamped& drone_pose, PosTarget* setpoint_p) {
    auto& drone_pos = drone_pose.pose.position;
    double distance_to_gb = sqrt(pow((gb_pose.x - drone_pos.x),2)
                                + pow((gb_pose.y - drone_pos.y),2));
    //Do checks here and transition to land
    if (distance_to_gb < DISTANCE_THRESHOLD && drone_pose.pose.position.y < HEIGHT_THRESHOLD && gb_pose.downward_tracked) {
        return LocalState::LAND;
    }
    else {
        return LocalState::RECOVER;
    }
}

LocalState landStateHandler(const GRstate& gb_pose, const PoseStamped& drone_pose, PosTarget* setpoint_p) {
    //Gets the landdetector.
    auto* land_detector = LandDetector::getSharedInstancePtr();

    //Runs the interception algorithm
    bool interception_ok = control::gb::interceptGB(drone_pose, gb_pose, *setpoint_p);
    
    //Checks if we have landed or the data is not good.
    if(land_detector->isOnGround()) {
        //Success
        return LocalState::RECOVER;
    }
    else if(!interception_ok) {
        //Oh shit!
        return LocalState::ABORT;
    }
    
    else {
        //Keep running the algorithm.
        return LocalState::LAND;
    }
}

LocalState recoverStateHandler(const GRstate& gb_pose, const PoseStamped& drone_pose, PosTarget* setpoint_p) {
    if (drone_pose.pose.position.z < HEIGHT_THRESHOLD) {
        // Setpoint to a safe altitude.
        // TODO This is probably not safe with Mist
        setpoint_p->type_mask = default_mask;
        setpoint_p->position.x = drone_pose.pose.position.x;  
        setpoint_p->position.y = drone_pose.pose.position.y;
        setpoint_p->position.z = control::Config::safe_hover_altitude;
        return LocalState::RECOVER;

    }
    return LocalState::COMPLETED;
}

LocalState abortStateHandler(const GRstate& gb_state, const PoseStamped& drone_pose, PosTarget* setpoint_p) {
    return LocalState::ABORT;
}

LocalState completedStateHandler(const GRstate& gb_pose, const PoseStamped& drone_pose, PosTarget* setpoint_p) {
    return LocalState::COMPLETED;
}

LandGBState::LandGBState() {
    setpoint_.type_mask = default_mask;
}

void LandGBState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidRequest()) {
        if(event.request == RequestType::ABORT) {
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
            //Transition to poshold to execute new cmd
            fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, event);
        }
    }
}

bool isValidGroundRobotID(int id, size_t num_gb) {
    return !(id < 0 || static_cast<size_t>(id) >= num_gb);
}

bool isValidTargetGB(const ascend_msgs::GRState& gb) {
    return gb.visible && gb.downward_tracked;
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

        const auto& gb_states = GroundRobotHandler::getCurrentGroundRobots();
        //Check gb_id
        if(!isValidGroundRobotID(cmd_.gb_id, gb_states.size())) {
            cmd_.eventError("Invalid GB id!");
            cmd_.clear();
            RequestEvent abort_event(RequestType::ABORT);
            fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
            return;
        }
        auto& gb = gb_states.at(static_cast<unsigned long>(cmd_.gb_id));
        if(!isValidTargetGB(gb)) {
            //Not possible to land on ground robot
            cmd_.eventError("LandGB not possible");
            control::handleWarnMsg("Land on ground robot not possible");
            RequestEvent abort_event(RequestType::ABORT);
            fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
            return;
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
    const auto& gb_array = control::GroundRobotHandler::getCurrentGroundRobots();
    
    ascend_msgs::GRState gb_pose = gb_array.at(static_cast<unsigned long>(cmd_.gb_id));
    auto& drone_pose = control::DroneHandler::getCurrentPose();

    PosTarget* setpoint_p = &setpoint_;

    // Loops the current and sets the next state.
    local_state = stateFunction(gb_pose, drone_pose, setpoint_p);

    // Sets the new state function
    stateFunction = state_function_array[static_cast<int>(local_state)];

    if (local_state == LocalState::COMPLETED) {
            cmd_.finishCMD();
            RequestEvent transition(RequestType::POSHOLD);
            fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, transition);        
    }
    if (local_state == LocalState::ABORT) {
            cmd_.abort();
            RequestEvent abort_event(RequestType::ABORT);
            fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
    }

}

const mavros_msgs::PositionTarget* LandGBState::getSetpointPtr() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

void LandGBState::handleManual(ControlFSM &fsm) {
    if(cmd_.isValidCMD()) {
        cmd_.eventError("OFFBOARD lost");
        cmd_.clear();
    }
    RequestEvent manual_event(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUAL_FLIGHT_STATE, this, manual_event);
}


ascend_msgs::ControlFSMState LandGBState::getStateMsg() const {
    using ascend_msgs::ControlFSMState;
    ControlFSMState msg;
    msg.name = getStateName();
    msg.state_id = ControlFSMState::INTERACT_GB_STATE;
    return msg;
}
