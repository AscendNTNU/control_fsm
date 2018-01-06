#include "control/tools/drone_handler.hpp"
#include "control/tools/logger.hpp"
#include "control/tools/config.hpp"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <control/tools/control_message.hpp>
#include <control/exceptions/ROSNotInitializedException.hpp>

using control::DroneHandler;

std::shared_ptr<DroneHandler> DroneHandler::shared_instance_p_ = nullptr;

DroneHandler::DroneHandler() {
    using control::Config;
    pose_sub_ = n_.subscribe(Config::mavros_local_pos_topic, 1, &DroneHandler::onPoseRecievedCB, this);
    state_sub_ = n_.subscribe(Config::mavros_state_changed_topic, 1, &DroneHandler::onStateRecievedCB, this);
}

std::shared_ptr<DroneHandler> DroneHandler::getSharedInstancePtr() {
    if(shared_instance_p_ == nullptr) {
        if(!ros::isInitialized()) {
            throw control::ROSNotInitializedException();
        }
        try {
            shared_instance_p_ = std::shared_ptr<DroneHandler>(new DroneHandler);
        } catch(const std::bad_alloc& e) {
            ROS_ERROR("[Control] Bad alloc exception");
            throw;
        }
    }
    return shared_instance_p_;
}

geometry_msgs::PoseStamped DroneHandler::getCurrentPose() {
    return getSharedInstancePtr()->getPose(); 
}

mavros_msgs::State DroneHandler::getCurrentState() {
    return getSharedInstancePtr()->getState();
}

geometry_msgs::PoseStamped DroneHandler::getPose() {
    if(control::message::hasTimedOut(last_pose_)) {
        control::handleErrorMsg("DroneHandler: Using old pose");
    }
    return last_pose_;
}

mavros_msgs::State DroneHandler::getState() {
    if(control::message::hasTimedOut(last_state_)) {
        control::handleErrorMsg("DroneHandler: Using old state"); 
    }
    return last_state_;
}

bool DroneHandler::isPoseValid() {
    return !control::message::hasTimedOut(getCurrentPose());
}

bool DroneHandler::isStateValid() {
    return !control::message::hasTimedOut(getCurrentState());
}
