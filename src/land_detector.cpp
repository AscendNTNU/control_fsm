//
// Created by haavard on 01.06.17.
//
#include "control_fsm/fsm_config.hpp"
#include "control_fsm/land_detector.hpp"
#include "control_fsm/control_fsm.hpp"

constexpr uint8_t LANDED_STATE_ON_GROUND = mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
std::shared_ptr<LandDetector> LandDetector::shared_instance_p_;

LandDetector::LandDetector() {
    assert(ros::isInitialized());
    sub_ = nh_.subscribe(FSMConfig::land_detector_topic, 1, &LandDetector::landCB, this);
}

void LandDetector::landCB(const mavros_msgs::ExtendedState &msg) {
    last_msg_ = msg;
}

bool LandDetector::isOnGround() {
    if(ros::Time::now() - last_msg_.header.stamp > ros::Duration(FSMConfig::valid_data_timeout)) {
        ROS_ERROR("[Control FSM] Using old land detector data!");
    }
    return last_msg_.landed_state == LANDED_STATE_ON_GROUND;
}

bool LandDetector::isReady() {
    //Make sure landing detector node is running.
    return sub_.getNumPublishers() > 0;
}

std::shared_ptr<LandDetector> LandDetector::getSharedInstancePtr() throw(std::bad_alloc) {
    if(shared_instance_p_ == nullptr) {
        try {
            shared_instance_p_ = std::shared_ptr<LandDetector>(new LandDetector);
        } catch (const std::bad_alloc& e) {
            ROS_ERROR("[Control FSM] Couldn't allocate memory for land detector!");
            throw;
        }
    }
    return shared_instance_p_;
}
