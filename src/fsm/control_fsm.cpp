#include "control/fsm/control_fsm.hpp"
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <control/fsm/control_fsm.hpp>
#include <control/tools/config.hpp>
#include <control/tools/logger.hpp>
#include <control/tools/obstacle_state_handler.hpp>

#ifndef PI_HALF
#define PI_HALF 1.57079632679
#endif

BeginState ControlFSM::BEGIN_STATE;
PreFlightState ControlFSM::PREFLIGHT_STATE;
IdleState ControlFSM::IDLE_STATE;
TakeoffState ControlFSM::TAKEOFF_STATE;
BlindHoverState ControlFSM::BLIND_HOVER_STATE;
PositionHoldState ControlFSM::POSITION_HOLD_STATE;
TrackGBState ControlFSM::TRACK_GB_STATE;
InteractGBState ControlFSM::INTERACT_GB_STATE;
GoToState ControlFSM::GO_TO_STATE;
LandState ControlFSM::LAND_STATE;
ManualFlightState ControlFSM::MANUAL_FLIGHT_STATE;

std::shared_ptr<ControlFSM> ControlFSM::shared_instance_p_;

//Change the current running state - be carefull to only change into an allowed state
void ControlFSM::transitionTo(StateInterface& state, StateInterface* caller_p, const EventData& event) {
    //Only current running state is allowed to change state
    if(getState() == caller_p) {
        //Run stateEnd on current running state before transitioning
        getState()->stateEnd(*this, event);
        //Set the current state pointer
        state_vault_.current_state_p_ = &state;
        control::handleInfoMsg("Current state: " + getState()->getStateName());
        //Pass event to new current state
        getState()->stateBegin(*this, event);
        //Notify state has changed
        on_state_changed_();
    } else {
        control::handleErrorMsg("Transition request made by not active state");
    }
}

//Send external event to current state and to "next" state
void ControlFSM::handleEvent(const EventData& event) {
    if(getState() == nullptr) {
        control::handleErrorMsg("Bad implementation of FSM - FSM allways need a state");
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
    assert(getState() != nullptr);
    getState()->loopState(*this);
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

    //Some checks can be skipped for debugging purposes
    if(control::Config::require_all_data_streams) {

        //Check that we're recieving position
        if(!control::DroneHandler::isPoseValid()) {
            control::handleWarnMsg("Missing position data");
            return false;
        }

        //Mavros must publish state data
        if (subscribers_.mavros_state_changed_sub.getNumPublishers() <= 0) {
            control::handleWarnMsg("Missing mavros state info!");
            return false;
        }
        try {
            //Land detector must be ready
            if (!LandDetector::getSharedInstancePtr()->isReady()) {
                control::handleWarnMsg("Missing land detector stream!");
                return false;
            }
        } catch(const std::exception& e) {
            control::handleErrorMsg("Exception: " + std::string(e.what()));
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
    RequestEvent event(RequestType::PREFLIGHT);
    transitionTo(PREFLIGHT_STATE, &BEGIN_STATE, event);
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

std::shared_ptr<ControlFSM> ControlFSM::getSharedInstancePtr() {

    if(!ros::isInitialized()) {
        throw control::ROSNotInitializedException();
    }

    if(shared_instance_p_ == nullptr) {
        try {
            shared_instance_p_ = std::shared_ptr<ControlFSM>(new ControlFSM);
        } catch(const std::bad_alloc& e) {
            std::string err_msg = "ControlFSM allocation failure: ";
            err_msg += e.what();
            control::handleErrorMsg(err_msg);
            throw;
        }
    }
    return shared_instance_p_;
}







