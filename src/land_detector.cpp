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
    lastMsg_ = msg;
}

bool LandDetector::isOnGround() {
    if(ros::Time::now() - lastMsg_.header.stamp > ros::Duration(FSMConfig::ValidDataTimeout)) {
        if(pFsm_ != nullptr) {
            pFsm_->handleFSMError("LandDetector using old data");
        } else {
            ROS_ERROR("LandDetector using old data!");
        }
    }
    return lastMsg_.value;
}

LandDetector::LandDetector(std::string topic, ControlFSM* pFsm) : LandDetector(topic) {
    pFsm_ = pFsm;
}

bool LandDetector::isReady() {
    //Make sure landing detector node is running.
    return sub_.getNumPublishers() > 0;
}
