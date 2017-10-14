//
// Created by haavard on 12.10.17.
//

#include <control_fsm/fsm_config.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "control_fsm/tools/control_pose.hpp"

constexpr float PI_HALF = 1.57079632679f;

//Static instance
std::shared_ptr<ControlPose> ControlPose::instance_;

bool checkAndReportMsgTimeout(const geometry_msgs::PoseStamped& msg) {
    if(ros::Time::now() - msg.header.stamp > ros::Duration(FSMConfig::valid_data_timeout)){
        //TODO Report error
        return true;
    }
    return false;
}

ControlPose::ControlPose() {
    pos_sub_ = n_.subscribe(FSMConfig::mavros_local_pos_topic, 1, &ControlPose::positionCB, this);
}

//TODO Write unit test
std::array<float, 3> ControlPose::getPositionXYZ() {
    checkAndReportMsgTimeout(last_position_);
    std::array<float, 3> temp{};
    auto& position = last_position_.pose.position;
    temp[0] = static_cast<float>(position.x); //From double to float
    temp[1] = static_cast<float>(position.y); //From double to float
    temp[2] = static_cast<float>(position.z); //From double to float
    return temp;
}

//TODO Write unit test
float ControlPose::getYaw() {
    checkAndReportMsgTimeout(last_position_);
    auto& orientation = last_position_.pose.orientation;
    double quat_x = orientation.x;
    double quat_y = orientation.y;
    double quat_z = orientation.z;
    double quat_w = orientation.w;
    tf2::Quaternion q(quat_x, quat_y, quat_z, quat_w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return static_cast<float>(yaw); //Explicitly casts to float
}

//TODO Write unit test
float ControlPose::getMavrosCorrectedYaw() {
    return getYaw() - PI_HALF;
}

//TODO Write unit test
std::array<float, 4> ControlPose::getOrientation() {
    checkAndReportMsgTimeout(last_position_);
    auto& orientation = last_position_.pose.orientation;
    auto quat_x = static_cast<float>(orientation.x);
    auto quat_y = static_cast<float>(orientation.y);
    auto quat_z = static_cast<float>(orientation.z);
    auto quat_w = static_cast<float>(orientation.w);
    std::array<float, 4> temp {quat_x, quat_y, quat_z, quat_w};
    return temp;
}

std::shared_ptr<ControlPose> ControlPose::getSharedPosePtr() {
    if(instance_ == nullptr) {
        //Only one instance exists - shared across states
        instance_ = std::shared_ptr<ControlPose>(new ControlPose());
    }
    return instance_; //Returns a copy of shared_ptr
}

bool ControlPose::isPoseValid() {
    bool isRecieving = pos_sub_.getNumPublishers() > 0;
    bool validMsg = !checkAndReportMsgTimeout(last_position_);
    return isRecieving && validMsg;
}
