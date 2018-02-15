#include "control/tools/obstacle_state_handler.hpp"
#include "control/exceptions/ros_not_initialized_exception.hpp"

using control::ObstacleStateHandler;

std::unique_ptr<ObstacleStateHandler> ObstacleStateHandler::shared_instance_p_ = nullptr;

ObstacleStateHandler::ObstacleStateHandler() : last_msg_p_(new ascend_msgs::GRStateArray) {
    using control::Config;
    std::string& topic = Config::obstacle_state_topic;
    obs_sub_ = n_.subscribe(topic.c_str(), 1, &ObstacleStateHandler::onMsgRecievedCB, this);
}

void ObstacleStateHandler::onMsgRecievedCB(ascend_msgs::GRStateArray::ConstPtr msg_p) {
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

const std::vector<ascend_msgs::GRState>& ObstacleStateHandler::getCurrentObstacles() {
    return getSharedObstacleHandlerPtr()->getObstacles();
}

const std::vector<ascend_msgs::GRState>& ObstacleStateHandler::getObstacles() const {
    isReady();
    return last_msg_p_->states;
}

bool ObstacleStateHandler::isInstanceReady() {
    return getSharedObstacleHandlerPtr()->isReady();
}

bool ObstacleStateHandler::isReady() const {
    if(last_msg_p_->states.empty()) {
        control::handleWarnMsg("Obstacle handler: No data available");
        return false;
    } else if(control::message::hasTimedOut(last_msg_p_->states[0])) {
        control::handleWarnMsg("Obstacle handler: Using old data");
        return false;
    }
    return true;
}


