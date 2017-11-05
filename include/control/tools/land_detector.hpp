//
// Created by haavard on 01.06.17.
//

#ifndef CONTROL_FSM_LANDDETECTOR_HPP
#define CONTROL_FSM_LANDDETECTOR_HPP

#include <ros/ros.h>
#include <mavros_msgs/ExtendedState.h>
#include <memory>
#include <control/exceptions/ROSNotInitializedException.hpp>

namespace {
using control::ROSNotInitializedException;
using std::bad_alloc;
}

class LandDetector {

private:
    ///Shared instance - singleton pattern
    static std::shared_ptr<LandDetector> shared_instance_p_;
    ///ROS Nodehandle
    ros::NodeHandle nh_;
    ///Subscriber for land detector messages
    ros::Subscriber sub_;
    ///Last recieved message
    mavros_msgs::ExtendedState last_msg_;
    ///Callback for last recieved message
    void landCB(const mavros_msgs::ExtendedState& msg);
    ///Private constructor
    LandDetector();

public:
    ///Is drone in the air or landed?
    bool isOnGround();
    ///Is data streams available and ready?
    bool isReady();
    ///Get shared_ptr to shared instance - singleton pattern
    static std::shared_ptr<LandDetector> getSharedInstancePtr();
};

#endif //CONTROL_FSM_LANDDETECTOR_HPP
