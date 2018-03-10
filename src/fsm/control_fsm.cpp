#include "control/fsm/control_fsm.hpp"
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <control/fsm/control_fsm.hpp>
#include <control/tools/config.hpp>
#include <control/tools/logger.hpp>
#include <control/tools/obstacle_state_handler.hpp>

BeginState ControlFSM::BEGIN_STATE;
PreFlightState ControlFSM::PREFLIGHT_STATE;
IdleState ControlFSM::IDLE_STATE;
TakeoffState ControlFSM::TAKEOFF_STATE;
BlindHoverState ControlFSM::BLIND_HOVER_STATE;
PositionHoldState ControlFSM::POSITION_HOLD_STATE;
LandGBState ControlFSM::LAND_GB_STATE;
GoToState ControlFSM::GO_TO_STATE;
LandState ControlFSM::LAND_STATE;
ManualFlightState ControlFSM::MANUAL_FLIGHT_STATE;

//Change the current running state - be carefull to only change into an allowed state
//Due to poor design, transitionTo has no strong nothrow guarantees - not exception safe!!
//Will lead to undefined behaviour if exception is thrown
void ControlFSM::transitionTo(StateInterface& state, StateInterface* caller_p, const EventData& event) {
    //Only current running state is allowed to change state
    if(getState() == caller_p) {
        //Run stateEnd on current running state before transitioning
        getState()->stateEnd(*this, event);
        //Set the current state pointer
        state_vault_.current_state_p_ = &state;
        control::handleInfoMsg("Current state: " + getState()->getStateName());
        //Notify state has changed
        on_state_changed_();
        //Pass event to new current state
        getState()->stateBegin(*this, event);
    } else {
        control::handleErrorMsg("Transition request made by not active state");
    }
}

//Send external event to current state and to "next" state
void ControlFSM::handleEvent(const EventData& event) {
    if(getState() == nullptr) {
        control::handleCriticalMsg("Bad implementation of FSM - FSM allways need a state");
        return;
    }
    if(event.event_type == EventType::MANUAL) {
        //If drone entered manual mode: Abort current operation, and go to stable.
        this->handleManual();
    }
    //Pass event to current running state
    getState()->handleEvent(*this, event);
}

//Runs state specific code on current state
void ControlFSM::loopCurrentState(void) {
    try {
        assert(getState() != nullptr);
        getState()->loopState(*this);
    } catch(const std::exception& e) {
        //If exceptions aren't handled by states - notify and try to go to blind hover
        //Will lead to undefined behaviour- but still safer than nothing!
        control::handleCriticalMsg(e.what());
        RequestEvent abort_event(RequestType::ABORT);
        transitionTo(BLIND_HOVER_STATE, getState(), abort_event);
    }
}

ControlFSM::ControlFSM() {
    //Set starting state
    state_vault_.current_state_p_ = &BEGIN_STATE;

    //Initialize all states
    this->initStates();

    //Subscribe to neccesary topics
    std::string& stateTopic = control::Config::mavros_state_changed_topic;
    subscribers_.mavros_state_changed_sub = node_handler_.subscribe(stateTopic, 1, &ControlFSM::mavrosStateChangedCB, this);
}

void ControlFSM::initStates() {
    //Only init once
    if(states_is_ready_) return;
    for(auto it = StateInterface::cbegin(); it != StateInterface::cend(); it++) {
        (*it)->stateInit(*this);
    }
    states_is_ready_ = true;
}

bool ControlFSM::isReady() {
    //Only check states if not already passed
    if(drone_state_.is_preflight_completed) return true;

    //All states must run their own checks
    for(auto it = StateInterface::cbegin(); it != StateInterface::cend(); it++) {
        control::handleInfoMsg((*it)->getStateName() + " is testing");
        if(!(*it)->stateIsReady(*this)) return false;
    }
    //Make sure obstacle avoidance is ready if enabled
    if(control::Config::require_obstacle_detection) {
        //Obstacle avoidance must be ready
        if(!obstacle_avoidance_.isReady()) {
            control::handleWarnMsg("Preflight Check: Obstacle avoidance not ready!");
            return false;
        }
    }

    //Some checks can be skipped for debugging purposes
    if(control::Config::require_all_data_streams) {
        try {
            //Check that we're recieving position
            if(!control::DroneHandler::isPoseValid()) {
                control::handleWarnMsg("Preflight Check: No valid pose data");
                return false;
            }
            //Mavros must publish state data
            if (subscribers_.mavros_state_changed_sub.getNumPublishers() <= 0) {
                control::handleWarnMsg("Preflight Check: No valid mavros state data!");
                return false;
            } 
            //Land detector must be ready
            if (!LandDetector::getSharedInstancePtr()->isReady()) {
                control::handleWarnMsg("Preflight Check: No valid land detector data!");
                return false;
            }
        } catch(const std::exception& e) {
            ///Critical bug -
            control::handleCriticalMsg(e.what());
            return false;
        }

        if(control::Config::require_obstacle_detection) {
            try {
                using control::ObstacleStateHandler;
                //Land detector must be ready
                if (!ObstacleStateHandler::isInstanceReady()) {
                    control::handleWarnMsg("Missing obstacle state stream!");
                    return false;
                }
            } catch(const std::exception& e) {
                control::handleErrorMsg("Exception: " + std::string(e.what()));
                return false;
            }
        }
    }

    //Preflight has passed - no need to check it again.
    drone_state_.is_preflight_completed = true;
    return true;
}

void ControlFSM::startPreflight() {
    if(!isReady()) {
        control::handleWarnMsg("FSM not ready, can't transition to preflight!");
        return;
    }
    if(getState() == &BEGIN_STATE) {
        RequestEvent event(RequestType::PREFLIGHT);
        transitionTo(PREFLIGHT_STATE, &BEGIN_STATE, event);
    } else {
        std::string err_msg = "Can't transition to preflight from ";
        err_msg += getState()->getStateName();
        control::handleErrorMsg(err_msg);
    }
}

void ControlFSM::mavrosStateChangedCB(const mavros_msgs::State &state) {
    bool offboard_true = (state.mode == std::string("OFFBOARD"));
    bool armed_true = (bool)state.armed;
    //Only act if relevant states has changed
    if(offboard_true != drone_state_.is_offboard || armed_true != drone_state_.is_armed) {
        //Check if old state was autonomous
        //=> now in manual mode
        if(drone_state_.is_offboard && drone_state_.is_armed) {
            ROS_INFO("Manual sent!");
            this->handleManual();
        }
        //Set current state
        drone_state_.is_offboard = offboard_true;
        drone_state_.is_armed = state.armed;

        //If it is armed and in offboard and all preflight checks has completed - notify AUTONOMOUS mode
        if(drone_state_.is_armed && drone_state_.is_offboard && drone_state_.is_preflight_completed) {
            EventData autonomousEvent;
            autonomousEvent.event_type = EventType::AUTONOMOUS;
            ROS_INFO("Autonomous event sent");
            this->handleEvent(autonomousEvent);
        }
    }
}

void ControlFSM::handleManual() {
    getState()->handleManual(*this);
}

mavros_msgs::PositionTarget ControlFSM::getMavrosSetpoint() {
    auto state_setpoint_p = getState()->getSetpointPtr();
    return obstacle_avoidance_.run(*state_setpoint_p);
}

