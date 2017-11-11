//
// Created by haavard on 03.11.17.
//

#include <control/exceptions/ROSNotInitializedException.hpp>
#include <utility>
#include <std_msgs/String.h>
#include <control/tools/config.hpp>
#include "control/tools/logger.hpp"

///Class responsible for publishing status messages to topics
class ROSTopicLogger {
private:
    static std::shared_ptr<ROSTopicLogger> shared_instance_p_;
    ros::NodeHandle nh_;
    ros::Publisher error_pub_;
    ros::Publisher warn_pub_;
    ros::Publisher info_pub_;
    ROSTopicLogger();
public:
    static std::shared_ptr<ROSTopicLogger> getSharedInstancePtr();
    void publishErrorMessage(std::string message);
    void publishWarnMessage(std::string message);
    void publishInfoMessage(std::string message);

};
///Static shared instance
std::shared_ptr<ROSTopicLogger> ROSTopicLogger::shared_instance_p_;

///Constructor initializes publishers
ROSTopicLogger::ROSTopicLogger() {
    using control::Config;
    error_pub_ = nh_.advertise<std_msgs::String>(Config::fsm_error_topic, Config::fsm_status_buffer_size);
    warn_pub_ = nh_.advertise<std_msgs::String>(Config::fsm_warn_topic, Config::fsm_status_buffer_size);
    info_pub_ = nh_.advertise<std_msgs::String>(Config::fsm_info_topic, Config::fsm_status_buffer_size);
}

///Returns shared_ptr to shared instance IF ros is initialized.
std::shared_ptr<ROSTopicLogger> ROSTopicLogger::getSharedInstancePtr() {
    if(shared_instance_p_ == nullptr) {
        if(!ros::isInitialized()) {
            return nullptr;
        }
        try {
            shared_instance_p_ = std::shared_ptr<ROSTopicLogger>(new ROSTopicLogger);
        } catch(const std::bad_alloc& e) {
            control::handleCriticalMsg(e.what());
            return nullptr;
        }
    }
    return shared_instance_p_;
}

///Publishes message to errorstream
void ROSTopicLogger::publishErrorMessage(std::string message) {
    std_msgs::String msg;
    msg.data = std::move(message);
    error_pub_.publish(msg);
}
///Published message to warnstream
void ROSTopicLogger::publishWarnMessage(std::string message) {
    std_msgs::String msg;
    msg.data = std::move(message);
    warn_pub_.publish(msg);
}
///Published message to info stream
void ROSTopicLogger::publishInfoMessage(std::string message) {
    std_msgs::String msg;
    msg.data = std::move(message);
    info_pub_.publish(msg);
}

///Prints error message to ROS_ERROR and publishes it if possible
void control::handleErrorMsg(std::string message) {
    if(ROSTopicLogger::getSharedInstancePtr() != nullptr) {
        ROSTopicLogger::getSharedInstancePtr()->publishErrorMessage(message);
    }
    ROS_ERROR("[Control]: %s", message.c_str());
}

///Prints warn message to ROS_WARN and publishes it if possible
void control::handleWarnMsg(std::string message) {
    if(ROSTopicLogger::getSharedInstancePtr() != nullptr) {
        ROSTopicLogger::getSharedInstancePtr()->publishWarnMessage(message);
    }
    ROS_WARN("[Control]: %s", message.c_str());
}

///Prints info message to ROS_INFO and publishes it if possible
void control::handleInfoMsg(std::string message) {
    if(ROSTopicLogger::getSharedInstancePtr() != nullptr) {
        ROSTopicLogger::getSharedInstancePtr()->publishInfoMessage(message);
    }
    ROS_INFO("[Control]: %s", message.c_str());
}

void control::handleCriticalMsg(const char* message) {
    ROS_ERROR("[Control] CRITICAL: %s", message);
}
