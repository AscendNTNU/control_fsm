#include "control/tools/obstacle_avoidance.hpp"
#include "control/tools/math.hpp"

bool control::ObstacleAvoidance::doObstacleAvoidance(mavros_msgs::PositionTarget* setpoint) {
    //TODO Implement obstacle avoidance

    //Return true if setpoint has been modified.
    return false;
}

void control::ObstacleAvoidance::onModified() {
    //Run all callbacks
    for(auto& cb_p : on_modified_cb_set_ ) {
        (*cb_p)();
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

void control::ObstacleAvoidance::removeOnModifiedCBPtr(const std::shared_ptr<std::function<void()> >& cb_p) {
    for(auto it = on_modified_cb_set_.begin(); it != on_modified_cb_set_.end(); ++it) {
        if(*it == cb_p) {
            on_modified_cb_set_.erase(it);
            //std::set - no duplicates
            return;
        }
    }
}
