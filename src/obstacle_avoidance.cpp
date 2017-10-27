#include "control_fsm/obstacle_avoidance.hpp"

//Static instance
std::shared_ptr<control::ObstacleAvoidance> control::ObstacleAvoidance::instance_p_;


bool control::ObstacleAvoidance::doObstacleAvoidance(mavros_msgs::PositionTarget* setpoint) {
    //TODO Implement obstacle avoidance

    //Return true if setpoint has been modified.
    return true;
}

void control::ObstacleAvoidance::onModified() {
    //Run all callbacks
    for(std::function<void()>& cb : on_modified_cb_vec_ ) {
        cb();
    }
}

mavros_msgs::PositionTarget control::ObstacleAvoidance::run(mavros_msgs::PositionTarget setpoint) {
    //If obstacle avoidance has altered the setpoint
    if(doObstacleAvoidance(&setpoint)) {
        //Notify states
        onModified();
    }
    return setpoint;
}

std::shared_ptr<control::ObstacleAvoidance> control::ObstacleAvoidance::getSharedInstancePtr() {
    //Create new instance if there isn't one
    if(instance_p_ == nullptr) {
        instance_p_ = std::shared_ptr<ObstacleAvoidance>(new ObstacleAvoidance);
    }
    return instance_p_;
}
