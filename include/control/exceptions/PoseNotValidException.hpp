//
// Created by haavard on 06.11.17.
//

#ifndef CONTROL_FSM_POSENOTVALIDEXCEPTION_HPP
#define CONTROL_FSM_POSENOTVALIDEXCEPTION_HPP

#include <stdexcept>
namespace control {
///Thrown when pose is available, but not valid
class PoseNotValidException : std::runtime_error {
public:
    PoseNotValidException() : std::runtime_error("Pose not valid") {}
};
}

#endif //CONTROL_FSM_POSENOTVALIDEXCEPTION_HPP
