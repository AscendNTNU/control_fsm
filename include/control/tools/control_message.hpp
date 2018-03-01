#ifndef CONTROL_MESSAGE_HPP
#define CONTROL_MESSAGE_HPP
#include <ros/ros.h>
#include "config.hpp"

namespace control {
namespace message {
///Caluclates time since messages was stamped - must be stamped message
template<typename T> ros::Duration timeSinceMsg(const T& msg) {
    return ros::Time::now() - msg.header.stamp;
}
///Is message to old - must be stamped type
template<typename T> bool hasTimedOut(const T& msg, ros::Duration timeout) {
    return timeSinceMsg(msg) > timeout;
}
///Is message to old using config value - must be stamped type
template<typename T> bool hasTimedOut(const T& msg) {
    using control::Config;
    return hasTimedOut(msg, ros::Duration(Config::valid_data_timeout));
}

template<typename T> bool hasWrongFrame(const T& msg, const std::string& frame_id) {
    return msg.header.frame_id != frame_id;
}

template<typename T> bool hasWrongLocalFrame(const T& msg) {
    using control::Config;
    return hasWrongFrame(msg, Config::local_frame_id);
}

template<typename T> bool hasWrongGlobalFrame(const T& msg) {
    using control::Config;
    return hasWrongFrame(msg, Config::global_frame_id);
}

}
}
#endif
