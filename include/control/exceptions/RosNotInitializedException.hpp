//
// Created by haavard on 03.11.17.
//

#ifndef CONTROL_FSM_ROSNOTINITIALIZEDEXCEPTION_HPP
#define CONTROL_FSM_ROSNOTINITIALIZEDEXCEPTION_HPP

#include <ros/exception.h>

class RosNotInitializedException : std::exception {
public:

    virtual const char* what() const throw() override {
        return "ROS is not initialized!";
    }
};

#endif //CONTROL_FSM_ROSNOTINITIALIZEDEXCEPTION_HPP
