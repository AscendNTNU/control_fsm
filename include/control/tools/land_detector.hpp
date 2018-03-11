//
// Created by haavard on 01.06.17.
//

#ifndef CONTROL_FSM_LANDDETECTOR_HPP
#define CONTROL_FSM_LANDDETECTOR_HPP

#include <ros/ros.h>
#include <mavros_msgs/ExtendedState.h>
#include <memory>
#include <control/exceptions/ros_not_initialized_exception.hpp>


class LandDetectorImpl;

class LandDetector {
private:
    ///Shared instance - singleton pattern
    static std::unique_ptr<LandDetectorImpl> impl_;
    ///Private constructor
    LandDetector() = default;
    /**Get shared_ptr to shared instance - singleton pattern
     * @throw control::ROSNotInitialized
     * @throw std::bad_alloc
     * @return shared_ptr to shared instance (singleton)
     */
    static const LandDetectorImpl* getImplPtr();

public:
    ///Is drone in the air or landed?
    static bool isOnGround();
    ///Is system ready?
    static bool isReady();
};

#endif //CONTROL_FSM_LANDDETECTOR_HPP
