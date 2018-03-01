#include "control/tools/drone_handler.hpp"
#include "control/tools/logger.hpp"
#include "control/tools/config.hpp"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <control/tools/control_message.hpp>
#include <control/exceptions/ros_not_initialized_exception.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using control::DroneHandler;

std::unique_ptr<DroneHandler> DroneHandler::shared_instance_p_ = nullptr;

DroneHandler::DroneHandler() :tf_listener_(tf_buffer_) {
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

const geometry_msgs::PoseStamped& DroneHandler::getCurrentLocalPose() {
    return getSharedInstancePtr()->getLocalPose();
}

geometry_msgs::PoseStamped DroneHandler::getCurrentGlobalPose() {
    return getSharedInstancePtr()->getGlobalPose();
}

const geometry_msgs::PoseStamped& DroneHandler::getLocalPose() const {
    if(control::message::hasTimedOut(*last_pose_)) {
        control::handleErrorMsg("DroneHandler: Using old pose");
    }
    return *last_pose_;
}

const geometry_msgs::TwistStamped& DroneHandler::getLocalTwist() const {
    if(control::message::hasTimedOut(*last_twist_)) {
        control::handleErrorMsg("DroneHandler: Using old twist");
    }
    return *last_twist_;
}

geometry_msgs::PoseStamped DroneHandler::getGlobalPose() const {
    using control::Config;
    auto& local_pose = getLocalPose();
    geometry_msgs::PoseStamped global_pose = local_pose;
    try {
        //Get transform
        auto tf = tf_buffer_.lookupTransform(Config::global_frame_id, local_pose.header.frame_id, ros::Time(0));
        //Apply transform
        tf2::doTransform(local_pose, global_pose, tf);
        //Return global
        return global_pose;
    } catch (const tf2::TransformException& e) {
        control::handleErrorMsg(e.what());
        return local_pose;
    }
}

bool DroneHandler::isTwistValid() {
    return !getSharedInstancePtr()->isLocalPoseValid();
}

bool DroneHandler::isGlobalPoseValid() {
    return isLocalPoseValid() && getSharedInstancePtr()->isTransformValid();
}

const geometry_msgs::TwistStamped &control::DroneHandler::getCurrentTwist() {
    return getSharedInstancePtr()->getLocalTwist();
}

bool control::DroneHandler::isTransformValid() const {
    return tf_buffer_.canTransform(Config::global_frame_id, getLocalPose().header.frame_id, ros::Time(0));
}

bool control::DroneHandler::isLocalPoseValid() {
    auto& pose = getCurrentLocalPose();
    if(control::message::hasTimedOut(pose)) return false;
    return !control::message::hasWrongLocalFrame(pose);
}


geometry_msgs::TransformStamped control::DroneHandler::getGlobal2LocalTf() {
    try {
        //Get transform
        auto& buffer = getSharedInstancePtr()->tf_buffer_;
        return buffer.lookupTransform(Config::local_frame_id, Config::global_frame_id, ros::Time(0));
    } catch (const tf2::TransformException& e) {
        control::handleErrorMsg(e.what());
        return geometry_msgs::TransformStamped();
    }
}
