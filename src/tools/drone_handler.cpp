#include "control/tools/drone_handler.hpp"
#include "control/tools/logger.hpp"
#include "control/tools/config.hpp"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <control/tools/control_message.hpp>
#include <control/exceptions/ros_not_initialized_exception.hpp>

using control::DroneHandler;

std::unique_ptr<DroneHandler> DroneHandler::shared_instance_p_ = nullptr;

DroneHandler::DroneHandler() {
    using control::Config;
    auto callback = &DroneHandler::onPoseRecievedCB;
    auto& topic = Config::mavros_local_pos_topic;
    pose_sub_ = n_.subscribe(topic, 1, callback, this);
}

const DroneHandler* DroneHandler::getSharedInstancePtr() {
    if(shared_instance_p_ == nullptr) {
        if(!ros::isInitialized()) {
            throw control::ROSNotInitializedException();
        }
        shared_instance_p_ = std::unique_ptr<DroneHandler>(new DroneHandler);
    }
    return shared_instance_p_.get();
}

const geometry_msgs::PoseStamped& DroneHandler::getCurrentPose() {
    return getSharedInstancePtr()->getPose(); 
}


const geometry_msgs::PoseStamped& DroneHandler::getPose() const {
    if(control::message::hasTimedOut(last_pose_)) {
        control::handleErrorMsg("DroneHandler: Using old pose");
    }
    return last_pose_;
}

bool DroneHandler::isPoseValid() {
    return !control::message::hasTimedOut(getCurrentPose());
}

