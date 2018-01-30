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
    using geometry_msgs::PoseStamped;
    using geometry_msgs::TwistStamped;
    pose_sub_ = n_.subscribe(Config::mavros_local_pos_topic, 1, &DroneHandler::onPoseRecievedCB, this);
    twist_sub_ = n_.subscribe(Config::mavros_local_vel_topic, 1, &DroneHandler::onTwistRecievedCB, this);
    //Init default, empty value
    last_pose_ = PoseStamped::ConstPtr(new PoseStamped);
    last_twist_ = TwistStamped::ConstPtr(new TwistStamped);
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
    if(control::message::hasTimedOut(*last_pose_)) {
        control::handleErrorMsg("DroneHandler: Using old pose");
    }
    return *last_pose_;
}

const geometry_msgs::TwistStamped& DroneHandler::getTwist() const {
    if(control::message::hasTimedOut(*last_twist_)) {
        control::handleErrorMsg("DroneHandler: Using old twist");
    }
    return *last_twist_;
}

bool DroneHandler::isPoseValid() {
    return !control::message::hasTimedOut(getCurrentPose());
}

bool DroneHandler::isTwistValid() {
    return !control::message::hasTimedOut(getCurrentTwist());
}

const geometry_msgs::TwistStamped &control::DroneHandler::getCurrentTwist() {
    return getSharedInstancePtr()->getTwist();
}
