//
// Created by haavard on 01.06.17.
//

#ifndef CONTROL_FSM_LANDDETECTOR_HPP
#define CONTROL_FSM_LANDDETECTOR_HPP

#include <ros/ros.h>
#include <mavros_msgs/ExtendedState.h>
#include <memory>

class LandDetector {

private:
    static std::shared_ptr<LandDetector> shared_instance_p_;
    std::string topic_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    mavros_msgs::ExtendedState last_msg_;
    void landCB(const mavros_msgs::ExtendedState& msg);
    LandDetector();

public:
    bool isOnGround();
    bool isReady();

    static std::shared_ptr<LandDetector> getSharedInstancePtr() throw(std::bad_alloc);
};

#endif //CONTROL_FSM_LANDDETECTOR_HPP
