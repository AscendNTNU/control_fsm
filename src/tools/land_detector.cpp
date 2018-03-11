//
// Created by haavard on 01.06.17.
//
#include <control/tools/logger.hpp>
#include "control/tools/config.hpp"
#include "control/tools/land_detector.hpp"
#include "control/fsm/control_fsm.hpp"
#include "control/tools/control_message.hpp"
#include <ascend_msgs/LandDetector.h>
#include <algorithm>

//Interface for land detector
class LandDetectorImpl {
public:
    virtual bool isOnGround() const = 0;
    virtual bool isReady() const = 0;
};

///LandDetector using mavros
class MavrosExtendedState : public LandDetectorImpl {
    
private:
    ///ROS Nodehandle
    ros::NodeHandle nh_;
    ///Subscriber for land detector messages
    ros::Subscriber sub_;
    ///Last recieved message
    mavros_msgs::ExtendedState last_msg_;
    ///Callback for last recieved message
    void landCB(const mavros_msgs::ExtendedState& msg) { last_msg_ = msg; }
public:
    ///Private constructor
    MavrosExtendedState();
    bool isOnGround() const override;
    bool isReady() const override;
};

///LandDetector using ascend_msgs::LandDetector msg
class LandingGearDetector : public LandDetectorImpl {
private:
    ///ROS Nodehandle
    ros::NodeHandle nh_;
    ///Subscriber for landing gear messages
    ros::Subscriber sub_;
    ///Last recieved message
    ascend_msgs::LandDetector last_msg_;
    ///Callback for last recieved message
    void landCB(const ascend_msgs::LandDetector& msg) { last_msg_ = msg; }
public:
    ///Private constructor
    LandingGearDetector();
    bool isOnGround() const override;
    bool isReady() const override;
};

///Invalid land detector type
class InvalidLandDetector : public LandDetectorImpl {
public:
    bool isOnGround() const override { control::handleErrorMsg("InvalidLandDetector"); return true; }
    bool isReady() const override { control::handleErrorMsg("InvalidLandDetector"); return false; }
};

std::unique_ptr<LandDetectorImpl> LandDetector::impl_ = nullptr;

MavrosExtendedState::MavrosExtendedState() {
    assert(ros::isInitialized());
    auto cb = &MavrosExtendedState::landCB;
    sub_ = nh_.subscribe(control::Config::land_detector_topic, 1, cb, this);
}

LandingGearDetector::LandingGearDetector() {
    assert(ros::isInitialized());
    auto cb = &LandingGearDetector::landCB;
    sub_ = nh_.subscribe(control::Config::land_detector_topic, 1, cb, this);
}

bool MavrosExtendedState::isOnGround() const {
    if(control::message::hasTimedOut(last_msg_)){
        control::handleErrorMsg("Land detector using old data");
    }
    using mavros_msgs::ExtendedState;
    constexpr uint8_t LANDED_STATE_ON_GROUND = ExtendedState::LANDED_STATE_ON_GROUND;
    return last_msg_.landed_state == LANDED_STATE_ON_GROUND;
}

bool MavrosExtendedState::isReady() const {
    //Make sure landing detector node is running.
    if(control::message::hasTimedOut(last_msg_)) return false;
    return sub_.getNumPublishers() > 0;
}


bool LandingGearDetector::isOnGround() const {
    if(control::message::hasTimedOut(last_msg_)) {
        control::handleErrorMsg("Land detector using old data");
    }
    using ascend_msgs::LandDetector;
    switch(last_msg_.state) {
        case LandDetector::LANDED:
            return true;
        case LandDetector::IN_AIR:
            return false;
        default:
            control::handleErrorMsg("Invalid land detector status!");
            return false;
    }
}

bool LandingGearDetector::isReady() const {
    //Make sure landing detector node is running.
    if(control::message::hasTimedOut(last_msg_)) return false;
    return sub_.getNumPublishers() > 0;
}


 

const LandDetectorImpl* LandDetector::getImplPtr() {
    if(impl_ == nullptr) {
        if(!ros::isInitialized()) {
            throw control::ROSNotInitializedException();
        }
        std::string type = control::Config::land_detector_type;
        //Transform to lower
        std::transform(type.begin(), type.end(), type.begin(), [](const char& l) {
            return tolower(l);
        });
        if(type == "mavros_extended_state") {
            impl_ = std::unique_ptr<LandDetectorImpl>(new MavrosExtendedState);
        } else if(type == "landing_gear") {
            impl_ = std::unique_ptr<LandDetectorImpl>(new LandingGearDetector); 
        } else {
            impl_ = std::unique_ptr<LandDetectorImpl>(new InvalidLandDetector);
        }
        
        
    }
    return impl_.get();
}

bool LandDetector::isOnGround() {
    return getImplPtr()->isOnGround();
}

bool LandDetector::isReady() {
    return getImplPtr()->isReady();
}


