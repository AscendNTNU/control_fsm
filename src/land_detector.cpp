//
// Created by haavard on 01.06.17.
//
#include "control_fsm/fsm_config.hpp"
#include "control_fsm/land_detector.hpp"
#include "control_fsm/control_fsm.hpp"

LandDetector::LandDetector(std::string topic) : _topic(topic) {
    assert(ros::isInitialized());
    _sub = _nh.subscribe(topic, 1, &LandDetector::landCB, this);
}

void LandDetector::landCB(const ascend_msgs::BoolStamped &msg) {
    _lastMsg = msg;
}

bool LandDetector::isOnGround() {
    if(ros::Time::now() - _lastMsg.header.stamp > ros::Duration(FSMConfig::ValidDataTimeout)) {
        if(_pFsm != nullptr) {
            _pFsm->handleFSMError("LandDetector using old data");
        } else {
            ROS_ERROR("LandDetector using old data!");
        }
    }
    return _lastMsg.value;
}

LandDetector::LandDetector(std::string topic, ControlFSM* pFsm) : LandDetector(topic) {
    _pFsm = pFsm;
}

bool LandDetector::isReady() {
    //Make sure landing detector node is running.
    return _sub.getNumPublishers() > 0;
}
