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
    using sensor_msgs::Range;
    pose_sub_ = n_.subscribe(Config::mavros_local_pos_topic, 1, &DroneHandler::onPoseRecievedCB, this);
    twist_sub_ = n_.subscribe(Config::mavros_local_vel_topic, 1, &DroneHandler::onTwistRecievedCB, this);
    raw_dist_sub_ = n_.subscribe(Config::mavros_distance_sensor_topic, 1, &DroneHandler::onDistRecievedCB, this);
    //Init default, empty value
    last_pose_ = PoseStamped::ConstPtr(new PoseStamped);
    last_twist_ = TwistStamped::ConstPtr(new TwistStamped);
    last_dist_ = Range::ConstPtr(new Range); 
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

geometry_msgs::PoseStamped DroneHandler::getCurrentLocalPose() {
    return getSharedInstancePtr()->getLocalPose();
}

sensor_msgs::Range DroneHandler::getCurrentDistance() {
    return getSharedInstancePtr()->getDistance();
}

geometry_msgs::PoseStamped DroneHandler::getCurrentGlobalPose() {
    using control::Config;
    auto& local_pose = getSharedInstancePtr()->getLocalPose();
    auto global_pose = local_pose;
    auto tf = getLocal2GlobalTf();
    tf2::doTransform(local_pose, global_pose, tf);
    return global_pose;
    
}

const geometry_msgs::PoseStamped& DroneHandler::getLocalPose() const {
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


bool DroneHandler::isTwistValid() {
    auto& twist = getSharedInstancePtr()->getTwist();
    if(control::message::hasTimedOut(twist)) return false;
    //Assumes twist is local
    return !control::message::hasWrongLocalFrame(twist);
}

bool DroneHandler::isGlobalPoseValid() {
    return isLocalPoseValid() && getSharedInstancePtr()->isTransformsValid();
}

geometry_msgs::TwistStamped control::DroneHandler::getCurrentTwist() {
    return getSharedInstancePtr()->getTwist();
}

bool control::DroneHandler::isTransformsValid() {
    if(!Config::use_global_transforms) return true;
    const auto& buffer = getSharedInstancePtr()->tf_buffer_;
    
    bool tf1 = buffer.canTransform(Config::global_frame_id, Config::local_frame_id, ros::Time(0));
    bool tf2 = buffer.canTransform(Config::local_frame_id, Config::global_frame_id, ros::Time(0));
    return tf1 && tf2;
}

bool control::DroneHandler::isLocalPoseValid() {
    auto& pose = getSharedInstancePtr()->getLocalPose();
    if(control::message::hasTimedOut(pose)) return false;
    return !control::message::hasWrongLocalFrame(pose);
}


geometry_msgs::TransformStamped control::DroneHandler::getGlobal2LocalTf() {
    using control::Config;
    if(!Config::use_global_transforms) {
        auto tf = geometry_msgs::TransformStamped();
        tf.header.stamp = ros::Time::now();
        tf.header.frame_id = Config::global_frame_id;
        tf.child_frame_id = Config::local_frame_id;
        //The transform should be empty; local = global
        return tf;
    }
    try {
        //Get transform
        auto& buffer = getSharedInstancePtr()->tf_buffer_;
        return buffer.lookupTransform(Config::local_frame_id, Config::global_frame_id, ros::Time(0));
    } catch (const tf2::TransformException& e) {
        control::handleErrorMsg(e.what());
        return geometry_msgs::TransformStamped();
    }
}

geometry_msgs::TransformStamped control::DroneHandler::getLocal2GlobalTf() {
    using control::Config;
    if(!Config::use_global_transforms) {
        auto tf = geometry_msgs::TransformStamped();
        tf.header.stamp = ros::Time::now();
        tf.header.frame_id = Config::local_frame_id;
        tf.child_frame_id = Config::global_frame_id;
        //The transform should be empty; local = global
        return tf;
    }
    try {
        //Get transform
        auto& buffer = getSharedInstancePtr()->tf_buffer_;
        return buffer.lookupTransform(Config::global_frame_id, Config::local_frame_id, ros::Time(0));
    } catch(const tf2::TransformException& e) {
        control::handleErrorMsg(e.what());
        return geometry_msgs::TransformStamped();
    }
}

bool control::DroneHandler::isDistValid() {
    auto& dist = getSharedInstancePtr()->getDistance();
    return !control::message::hasTimedOut(dist);
}

const sensor_msgs::Range& control::DroneHandler::getDistance() const {
    if(control::message::hasTimedOut(*last_dist_)) {
        control::handleErrorMsg("DroneHandler: Using old distance");
    }
    return *last_dist_;
}

