#include "control/tools/obstacle_state_handler.hpp"
#include "control/exceptions/ROSNotInitializedException.hpp"

using control::ObstacleStateHandler;

std::unique_ptr<ObstacleStateHandler> ObstacleStateHandler::shared_instance_p_ = nullptr;

ObstacleStateHandler::ObstacleStateHandler() {
    using control::Config;
    std::string& topic = Config::obstacle_state_topic;
    obs_sub_ = n_.subscribe(topic.c_str(), 1, &ObstacleStateHandler::onMsgRecievedCB, this);
}

void ObstacleStateHandler::onMsgRecievedCB(const ascend_msgs::GRStateArray& msg) {
    last_msg_ = msg;
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

std::vector<ascend_msgs::GRState> ObstacleStateHandler::getCurrentObstacles() {
    return getSharedObstacleHandlerPtr()->last_msg_.states;
}

std::vector<ascend_msgs::GRState> ObstacleStateHandler::getObstacles() const {
    if(last_msg_.states.empty()) {
        control::handleErrorMsg("Obstacle handler: No data recieved");
    } else if(control::message::hasTimedOut(last_msg_.states[0])) {
        control::handleErrorMsg("Obstacle handler: Using old data");
    }
    return last_msg_.states;
}

bool ObstacleStateHandler::isInstanceReady() {
    return getSharedObstacleHandlerPtr()->isReady();
}

bool ObstacleStateHandler::isReady() const {
    if(last_msg_.states.empty()) {
        control::handleWarnMsg("Obstacle handler: No data available");
        return false;
    } else if(control::message::hasTimedOut(last_msg_.states[0])) {
        control::handleWarnMsg("Obstacle handler: Using old data");
        return false;
    }
    return true;
}


