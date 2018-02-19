//
// Created by haavard on 19.02.18.
//

#include <control/tools/config.hpp>
#include <control/exceptions/ros_not_initialized_exception.hpp>
#include <control/tools/logger.hpp>
#include <control/tools/control_message.hpp>
#include "control/tools/ground_robot_handler.hpp"

using control::GroundRobotHandler;
using control::Config;
using ascend_msgs::AIWorldObservation;

std::unique_ptr<GroundRobotHandler> GroundRobotHandler::shared_instance_p_ = nullptr;

GroundRobotHandler::GroundRobotHandler() : last_gb_msg_p_(new AIWorldObservation) {
    auto cb = boost::bind(&GroundRobotHandler::gbCB, this, _1);
    auto& topic = Config::ground_robot_state_topic;
    gb_sub_ = n_.subscribe<AIWorldObservation>(topic, 1, cb);
}

const GroundRobotHandler *control::GroundRobotHandler::getSharedInstancePtr() {
    if(shared_instance_p_ == nullptr) {
        if(!ros::isInitialized()) {
            throw control::ROSNotInitializedException();
        }
        shared_instance_p_ = std::unique_ptr<GroundRobotHandler>(new GroundRobotHandler);

    }
    return shared_instance_p_.get();
}

const GroundRobotHandler::GBVectorType& GroundRobotHandler::getGroundRobots() const {
    isReady();
    return last_gb_msg_p_->ground_robots;
}

bool GroundRobotHandler::isReady() const {
    if(control::message::hasTimedOut(*last_gb_msg_p_)) {
        control::handleWarnMsg("Obstacle handler: Using old data");
        return false;
    }
    return true;
}




