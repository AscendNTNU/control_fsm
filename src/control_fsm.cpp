#include "control_fsm/control_fsm.hpp"
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <control_fsm/control_fsm.hpp>
#include <control_fsm/fsm_config.hpp>

#ifndef PI_HALF
#define PI_HALF 1.57079632679
#endif

BeginState ControlFSM::BEGINSTATE;
PreFlightState ControlFSM::PREFLIGHTSTATE;
IdleState ControlFSM::IDLESTATE;
TakeoffState ControlFSM::TAKEOFFSTATE;
BlindHoverState ControlFSM::BLINDHOVERSTATE;
PositionHoldState ControlFSM::POSITIONHOLDSTATE;
ShutdownState ControlFSM::SHUTDOWNSTATE;
EstimateAdjustState ControlFSM::ESTIMATEADJUSTSTATE;
TrackGBState ControlFSM::TRACKGBSTATE;
InteractGBState ControlFSM::INTERACTGBSTATE;
GoToState ControlFSM::GOTOSTATE;
LandState ControlFSM::LANDSTATE;
BlindLandState ControlFSM::BLINDLANDSTATE;
ManualFlightState ControlFSM::MANUALFLIGHTSTATE;
bool ControlFSM::is_used = false;

//Change the current running state - be carefull to only change into an allowed state
void ControlFSM::transitionTo(StateInterface& state, StateInterface* pCaller, const EventData& event) {
    //Only current running state is allowed to change state
    if(getState() == pCaller) {
        //Run stateEnd on current running state before transitioning
        getState()->stateEnd(*this, event);
        //Set the current state pointer
        state_vault_.p_current_state_ = &state;
        handleFSMInfo("Current state: " + getState()->getStateName());
        //Pass event to new current state
        getState()->stateBegin(*this, event);
        //Notify state has changed
        on_state_changed__();
    } else {
        handleFSMError("Transition request made by not active state");
    }
}

//Send external event to current state and to "next" state
void ControlFSM::handleEvent(const EventData& event) {
    if(getState() == nullptr) {
        handleFSMError("Bad implementation of FSM - FSM allways need a state");
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

//Send error message to user via ROS
void ControlFSM::handleFSMError(std::string errMsg) {
    ROS_ERROR("%s", (std::string("[Control FSM] ") + errMsg).c_str());
    on_fsm_error_(errMsg);
}

//Send info message to user via ROS
void ControlFSM::handleFSMInfo(std::string infoMsg) {
    ROS_INFO("%s",(std::string("[Control FSM] ") + infoMsg).c_str());
    on_fsm_info_(infoMsg);
}

//Send warning to user via ROS
void ControlFSM::handleFSMWarn(std::string warnMsg) {
    ROS_WARN("%s", (std::string("[Control FSM] ") + warnMsg).c_str());
    on_fsm_warn_(warnMsg);
}

//Send debug message to user via ROS
void ControlFSM::handleFSMDebug(std::string debugMsg) {
    ROS_DEBUG("%s", (std::string("[Control FSM] ") + debugMsg).c_str());
}

const geometry_msgs::PoseStamped* ControlFSM::getPositionXYZ() {
    if(!drone_position_.is_set) {
        handleFSMError("Position has not been set!!");
        return nullptr;
    }
    auto& stamp = drone_position_.position.header.stamp;
    if(ros::Time::now() - stamp > ros::Duration(FSMConfig::valid_data_timeout)) {
        handleFSMError("Using old position data!");
    }
    return drone_position_.valid_xy ? &drone_position_.position : nullptr;
}

double ControlFSM::getOrientationYaw() {
    double quatX = drone_position_.position.pose.orientation.x;
    double quatY = drone_position_.position.pose.orientation.y;
    double quatZ = drone_position_.position.pose.orientation.z;
    double quatW = drone_position_.position.pose.orientation.w;
    tf2::Quaternion q(quatX, quatY, quatZ, quatW);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    //Subtracting PI halfs to correct for a bug in mavros (90 degree offset)
    return yaw;
}

double ControlFSM::getMavrosCorrectedYaw() {
    return getOrientationYaw() - PI_HALF;
}

//Returns only altitude
double ControlFSM::getPositionZ() {
    if(!drone_position_.is_set) {
        handleFSMError("Position has not been set!!");
    }
     return drone_position_.position.pose.position.z;
}

ControlFSM::ControlFSM() : land_detector_(FSMConfig::land_detector_topic, this) {
    //Only one instance of ControlFSM is allowed
    assert(!ControlFSM::is_used);
    //ROS must be initialized!
    assert(ros::isInitialized());
    //Set starting state
    state_vault_.p_current_state_ = &BEGINSTATE;

    //Initialize all states
    this->initStates();

    //Subscribe to neccesary topics
    std::string& posTopic = FSMConfig::mavros_local_pos_topic;
    std::string& stateTopic = FSMConfig::mavros_state_changed_topic;
    subscribers_.local_pos_sub = node_handler_.subscribe(posTopic, 1, &ControlFSM::localPosCB, this);
    subscribers_.mavros_state_changed_sub = node_handler_.subscribe(stateTopic, 1, &ControlFSM::mavrosStateChangedCB, this);

    //Make sure no other instances of ControlFSM is allowed
    ControlFSM::is_used = true;
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
        this->handleFSMInfo((*it)->getStateName() + "is testing");
        if(!(*it)->stateIsReady(*this)) return false;
    }

    //Some checks can be skipped for debugging purposes
    if(FSMConfig::require_all_data_streams) {
        //First position has been recieved
        if (!drone_position_.is_set) {
            this->handleFSMWarn("Missing initial position!");
            return false;
        }
        //Mavros must publish state data
        if (subscribers_.mavros_state_changed_sub.getNumPublishers() <= 0) {
            this->handleFSMWarn("Missing mavros state info!");
            return false;
        }
        //Mavros must publish position data
        if (subscribers_.local_pos_sub.getNumPublishers() <= 0) {
            this->handleFSMWarn("Missing local position stream!");
            return false;
        }
        //Land detector must be ready
        if (!land_detector_.isReady()) {
            this->handleFSMWarn("Missing land detector stream!");
            return false;
        }
    }

    //Preflight has passed - no need to check it again.
    drone_state_.is_preflight_completed = true;
    return true;
}

void ControlFSM::startPreflight() {
    if(!isReady()) {
        this->handleFSMWarn("FSM not ready, can't transition to preflight!");
        return;
    }
    RequestEvent event(RequestType::PREFLIGHT);
    transitionTo(PREFLIGHTSTATE, &BEGINSTATE, event);
}

void ControlFSM::localPosCB(const geometry_msgs::PoseStamped &input) {
    drone_position_.is_set = true;
    drone_position_.position = input;
}

void ControlFSM::mavrosStateChangedCB(const mavros_msgs::State &state) {
    bool offboardTrue = (state.mode == std::string("OFFBOARD"));
    bool armedTrue = (bool)state.armed;
    //Only act if relevant states has changed
    if(offboardTrue != drone_state_.is_offboard || armedTrue != drone_state_.is_armed) {
        //Check if old state was autonomous
        //=> now in manual mode
        if(drone_state_.is_offboard && drone_state_.is_armed) {
            ROS_INFO("Manual sent!");
            this->handleManual();
        }
        //Set current state
        drone_state_.is_offboard = offboardTrue;
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






