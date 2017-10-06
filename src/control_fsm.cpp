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
TrackGBState ControlFSM::TRACKGBSTATE;
InteractGBState ControlFSM::INTERACTGBSTATE;
GoToState ControlFSM::GOTOSTATE;
LandState ControlFSM::LANDSTATE;
ManualFlightState ControlFSM::MANUALFLIGHTSTATE;
bool ControlFSM::isUsed = false;

//Change the current running state - be carefull to only change into an allowed state
void ControlFSM::transitionTo(StateInterface& state, StateInterface* pCaller, const EventData& event) {
    //Only current running state is allowed to change state
    if(getState() == pCaller) {
        //Run stateEnd on current running state before transitioning
        getState()->stateEnd(*this, event);
        //Set the current state pointer
        stateVault_.pCurrentState_ = &state;
        handleFSMInfo("Current state: " + getState()->getStateName());
        //Pass event to new current state
        getState()->stateBegin(*this, event);
        //Notify state has changed
        onStateChanged_();
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
    if(event.eventType == EventType::MANUAL) {
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
    onFSMError_(errMsg);
}

//Send info message to user via ROS
void ControlFSM::handleFSMInfo(std::string infoMsg) {
    ROS_INFO("%s",(std::string("[Control FSM] ") + infoMsg).c_str());
    onFSMInfo_(infoMsg);
}

//Send warning to user via ROS
void ControlFSM::handleFSMWarn(std::string warnMsg) {
    ROS_WARN("%s", (std::string("[Control FSM] ") + warnMsg).c_str());
    onFSMWarn_(warnMsg);
}

//Send debug message to user via ROS
void ControlFSM::handleFSMDebug(std::string debugMsg) {
    ROS_DEBUG("%s", (std::string("[Control FSM] ") + debugMsg).c_str());
}

const geometry_msgs::PoseStamped* ControlFSM::getPositionXYZ() {
    if(!dronePosition_.isSet) {
        handleFSMError("Position has not been set!!");
        return nullptr;
    }
    auto& stamp = dronePosition_.position.header.stamp;
    if(ros::Time::now() - stamp > ros::Duration(FSMConfig::ValidDataTimeout)) {
        handleFSMError("Using old position data!");
    }
    return dronePosition_.validXY ? &dronePosition_.position : nullptr;
}

double ControlFSM::getOrientationYaw() {
    double quatX = dronePosition_.position.pose.orientation.x;
    double quatY = dronePosition_.position.pose.orientation.y;
    double quatZ = dronePosition_.position.pose.orientation.z;
    double quatW = dronePosition_.position.pose.orientation.w;
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
    if(!dronePosition_.isSet) {
        handleFSMError("Position has not been set!!");
    }
     return dronePosition_.position.pose.position.z;
}

ControlFSM::ControlFSM() : landDetector_(FSMConfig::LandDetectorTopic, this) {
    //Only one instance of ControlFSM is allowed
    assert(!ControlFSM::isUsed);
    //ROS must be initialized!
    assert(ros::isInitialized());
    //Set starting state
    stateVault_.pCurrentState_ = &BEGINSTATE;

    //Initialize all states
    this->initStates();

    //Subscribe to neccesary topics
    std::string& posTopic = FSMConfig::MavrosLocalPosTopic;
    std::string& stateTopic = FSMConfig::MavrosStateChangedTopic;
    subscribers_.localPosSub = nodeHandler_.subscribe(posTopic, 1, &ControlFSM::localPosCB, this);
    subscribers_.mavrosStateChangedSub = nodeHandler_.subscribe(stateTopic, 1, &ControlFSM::mavrosStateChangedCB, this);

    //Make sure no other instances of ControlFSM is allowed
    ControlFSM::isUsed = true;
}

void ControlFSM::initStates() {
    //Only init once
    if(statesIsReady_) return;
    for(auto it = StateInterface::cbegin(); it != StateInterface::cend(); it++) {
        (*it)->stateInit(*this);
    }
    statesIsReady_ = true;
}

bool ControlFSM::isReady() {
    //Only check states if not already passed
    if(droneState_.isPreflightCompleted) return true;

    //All states must run their own checks
    for(auto it = StateInterface::cbegin(); it != StateInterface::cend(); it++) {
        this->handleFSMInfo((*it)->getStateName() + "is testing");
        if(!(*it)->stateIsReady(*this)) return false;
    }

    //Some checks can be skipped for debugging purposes
    if(FSMConfig::RequireAllDataStreams) {
        //First position has been recieved
        if (!dronePosition_.isSet) {
            this->handleFSMWarn("Missing initial position!");
            return false;
        }
        //Mavros must publish state data
        if (subscribers_.mavrosStateChangedSub.getNumPublishers() <= 0) {
            this->handleFSMWarn("Missing mavros state info!");
            return false;
        }
        //Mavros must publish position data
        if (subscribers_.localPosSub.getNumPublishers() <= 0) {
            this->handleFSMWarn("Missing local position stream!");
            return false;
        }
        //Land detector must be ready
        if (!landDetector_.isReady()) {
            this->handleFSMWarn("Missing land detector stream!");
            return false;
        }
    }

    //Preflight has passed - no need to check it again.
    droneState_.isPreflightCompleted = true;
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
    dronePosition_.isSet = true;
    dronePosition_.position = input;
}

void ControlFSM::mavrosStateChangedCB(const mavros_msgs::State &state) {
    bool offboardTrue = (state.mode == std::string("OFFBOARD"));
    bool armedTrue = (bool)state.armed;
    //Only act if relevant states has changed
    if(offboardTrue != droneState_.isOffboard || armedTrue != droneState_.isArmed) {
        //Check if old state was autonomous
        //=> now in manual mode
        if(droneState_.isOffboard && droneState_.isArmed) {
            ROS_INFO("Manual sent!");
            this->handleManual();
        }
        //Set current state
        droneState_.isOffboard = offboardTrue;
        droneState_.isArmed = state.armed;

        //If it is armed and in offboard and all preflight checks has completed - notify AUTONOMOUS mode
        if(droneState_.isArmed && droneState_.isOffboard && droneState_.isPreflightCompleted) {
            EventData autonomousEvent;
            autonomousEvent.eventType = EventType::AUTONOMOUS;
            ROS_INFO("Autonomous event sent");
            this->handleEvent(autonomousEvent);
        }
    }
}

void ControlFSM::handleManual() {
    getState()->handleManual(*this);
}






