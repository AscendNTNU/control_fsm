//
// Created by haavard on 03.11.17.
//

#ifndef CONTROL_FSM_ROSNOTINITIALIZEDEXCEPTION_HPP
#define CONTROL_FSM_ROSNOTINITIALIZEDEXCEPTION_HPP

#include <ros/exception.h>
namespace control {
class ROSNotInitializedException : std::runtime_error {
public:
    ROSNotInitializedException() : std::runtime_error("ROS is not initialized!") {}
};
}

#endif //CONTROL_FSM_ROSNOTINITIALIZEDEXCEPTION_HPP
