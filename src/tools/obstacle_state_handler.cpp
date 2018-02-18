#include "control/tools/obstacle_state_handler.hpp"
#include "control/exceptions/ros_not_initialized_exception.hpp"

using control::ObstacleStateHandler;
using ascend_msgs::DetectedRobotsGlobalPositions;

std::unique_ptr<ObstacleStateHandler> ObstacleStateHandler::shared_instance_p_ = nullptr;

ObstacleStateHandler::ObstacleStateHandler() : last_msg_p_(new DetectedRobotsGlobalPositions) {
    using control::Config;
    std::string& topic = Config::lidar_topic;
    obs_sub_ = n_.subscribe(topic.c_str(), 1, &ObstacleStateHandler::onMsgRecievedCB, this);
}

void ObstacleStateHandler::onMsgRecievedCB(DetectedRobotsGlobalPositions::ConstPtr msg_p) {
    last_msg_p_ = msg_p;
}

const ObstacleStateHandler* ObstacleStateHandler::getSharedObstacleHandlerPtr() {
    if(shared_instance_p_ == nullptr) {
        if(!ros::isInitialized()) {
            throw control::ROSNotInitializedException();
        }
        shared_instance_p_ = std::unique_ptr<ObstacleStateHandler>(new ObstacleStateHandler);
        
    }
    return shared_instance_p_.get();
}

const DetectedRobotsGlobalPositions& ObstacleStateHandler::getCurrentObstacles() {
    return getSharedObstacleHandlerPtr()->getObstacles();
}

const DetectedRobotsGlobalPositions& ObstacleStateHandler::getObstacles() const {
    isReady();
    return *last_msg_p_;
}

bool ObstacleStateHandler::isInstanceReady() {
    return getSharedObstacleHandlerPtr()->isReady();
}

bool ObstacleStateHandler::isReady() const {
    if(last_msg_p_->count == 0) {
        control::handleWarnMsg("Obstacle handler: No data available");
        return false;
    } else if(control::message::hasTimedOut(*last_msg_p_)) {
        control::handleWarnMsg("Obstacle handler: Using old data");
        return false;
    }
    return true;
}


