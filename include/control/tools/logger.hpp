
#ifndef CONTROL_FSM_LOGGER_HPP
#define CONTROL_FSM_LOGGER_HPP
#include <ros/ros.h>

namespace control {
///Prints error message to ROS_ERROR and publishes it if possible
void handleErrorMsg(std::string message);
///Prints warning message to ROS_WARN and publishes it if possible
void handleWarnMsg(std::string message);
///Prints info message to ROS_INFO and publishes it if possible
void handleInfoMsg(std::string message);
///Prints critical message - even std::string is disallowed
void handleCriticalMsg(const char* message);
}

#endif //CONTROL_FSM_LOGGER_HPP
