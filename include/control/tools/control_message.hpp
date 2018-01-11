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

}
}
#endif
