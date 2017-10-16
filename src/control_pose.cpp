//
// Created by haavard on 12.10.17.
//

#include <control_fsm/fsm_config.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "control_fsm/tools/control_pose.hpp"

constexpr double MAVROS_YAW_CORRECTION_PI_HALF = 3.141592653589793 / 2.0;

using control::Pose;

//Static instance
std::shared_ptr<Pose> Pose::instance_;

bool checkAndReportMsgTimeout(const geometry_msgs::PoseStamped& msg) {
    if(ros::Time::now() - msg.header.stamp > ros::Duration(FSMConfig::valid_data_timeout)){
        //TODO Report error
        return true;
    }
    return false;
}

Pose::Pose() {
    pos_sub_ = n_.subscribe(FSMConfig::mavros_local_pos_topic, 1, &Pose::positionCB, this);
}

//TODO Write unit test
control::Point Pose::getPositionXYZ() {
    checkAndReportMsgTimeout(last_position_);
    Point temp;
    auto& position = last_position_.pose.position;
    temp.x = position.x; //From double to float
    temp.y = position.y; //From double to float
    temp.z = position.z; //From double to float
    return temp;
}

//TODO Write unit test
double Pose::getYaw() {
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
    return yaw; //Explicitly casts to float
}

//TODO Write unit test
double Pose::getMavrosCorrectedYaw() {
    return getYaw() - MAVROS_YAW_CORRECTION_PI_HALF;
}

//TODO Write unit test
control::Quaternion Pose::getOrientation() {
    checkAndReportMsgTimeout(last_position_);
    auto& orientation = last_position_.pose.orientation;
    auto quat_x = orientation.x;
    auto quat_y = orientation.y;
    auto quat_z = orientation.z;
    auto quat_w = orientation.w;
    Quaternion temp {quat_x, quat_y, quat_z, quat_w};
    return temp;
}

std::shared_ptr<Pose> Pose::getSharedPosePtr() {
    if(instance_ == nullptr) {
        //Only one instance exists - shared across states
        instance_ = std::shared_ptr<Pose>(new Pose());
    }
    return instance_; //Returns a copy of shared_ptr
}

bool Pose::isPoseValid() {
    bool isRecieving = pos_sub_.getNumPublishers() > 0;
    bool validMsg = !checkAndReportMsgTimeout(last_position_);
    return isRecieving && validMsg;
}
