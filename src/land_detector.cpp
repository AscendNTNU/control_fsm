//
// Created by haavard on 01.06.17.
//
#include "control_fsm/fsm_config.hpp"
#include "control_fsm/land_detector.hpp"
#include "control_fsm/control_fsm.hpp"

LandDetector::LandDetector(std::string topic) : topic_(topic) {
    assert(ros::isInitialized());
    sub_ = nh_.subscribe(topic, 1, &LandDetector::landCB, this);
}

void LandDetector::landCB(const ascend_msgs::BoolStamped &msg) {
    last_msg_ = msg;
}

bool LandDetector::isOnGround() {
    if(ros::Time::now() - last_msg_.header.stamp > ros::Duration(FSMConfig::valid_data_timeout)) {
        if(p_fsm_ != nullptr) {
            p_fsm_->handleFSMError("LandDetector using old data");
        } else {
            ROS_ERROR("LandDetector using old data!");
        }
    }
    return last_msg_.value;
}

LandDetector::LandDetector(std::string topic, ControlFSM* p_fsm) : LandDetector(topic) {
    p_fsm_ = p_fsm;
}

bool LandDetector::isReady() {
    //Make sure landing detector node is running.
    return sub_.getNumPublishers() > 0;
}
