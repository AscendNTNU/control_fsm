//
// Created by haavard on 12.10.17.
//

#include <control_fsm/fsm_config.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "control_fsm/tools/control_pose.hpp"

constexpr float PI_HALF = 1.57079632679;

ControlPose::ControlPose() {
    pos_sub_ = n_.subscribe(FSMConfig::mavros_local_pos_topic, 1, &ControlPose::positionCB, this);
}

std::array<float, 3> ControlPose::get_position_xyz() {
    std::array<float, 3> temp{};
    auto& position = last_position_.pose.position;
    temp[0] = static_cast<float>(position.x); //From double to float
    temp[1] = static_cast<float>(position.y); //From double to float
    temp[2] = static_cast<float>(position.z); //From double to float
    return temp;
}

float ControlPose::get_yaw() {
    auto& orientation = last_position_.pose.orientation;
    double quat_x = orientation.x;
    double quat_y = orientation.y;
    double quat_z = orientation.z;
    double quat_w = orientation.w;
    tf2::Quaternion q(quat_x, quat_y, quat_z, quat_w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    //Subtracting PI halfs to correct for a bug in mavros (90 degree offset)
    return yaw;
}

float ControlPose::get_mavros_corrected_yaw() {
    return get_yaw() - PI_HALF;
}

std::array<float, 4> ControlPose::get_orientation() {
    auto& orientation = last_position_.pose.orientation;
    double quat_x = orientation.x;
    double quat_y = orientation.y;
    double quat_z = orientation.z;
    double quat_w = orientation.w;
    std::array<float, 4> temp {quat_x, quat_y, quat_z, quat_w};
    return temp;
}
