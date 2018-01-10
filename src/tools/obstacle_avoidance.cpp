#include "control/tools/obstacle_avoidance.hpp"
#include "control/tools/obstacle_state_handler.hpp"
#include "control/tools/drone_handler.hpp"

#include <ascend_msgs/GRStateArray.h>
#include <ascend_msgs/GRState.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

//Static instance
std::shared_ptr<control::ObstacleAvoidance> control::ObstacleAvoidance::instance_p_;

constexpr float PI{3.14159265359f};

bool control::ObstacleAvoidance::doObstacleAvoidance(mavros_msgs::PositionTarget* setpoint) {
    bool setpoint_modified = false;
    
    const std::vector<ascend_msgs::GRState> obstacles = control::ObstacleStateHandler::getCurrentObstacles();
    const geometry_msgs::PoseStamped drone_pose = control::DroneHandler::getCurrentPose();

    for (const auto& obstacle : obstacles){

        geometry_msgs::Vector3 delta_drone_obstacle;
        delta_drone_obstacle.x = obstacle.x - drone_pose.pose.position.x;
        delta_drone_obstacle.y = obstacle.y - drone_pose.pose.position.y; 
        delta_drone_obstacle.z = drone_pose.pose.position.z; 
        const float distance_to_obstacle = std::sqrt(std::pow(delta_drone_obstacle.x, 2) + std::pow(delta_drone_obstacle.y, 2));
        if (distance_to_obstacle < clearance_max){
            // perform obstacle avoidance
            const float angle = std::atan2(delta_drone_obstacle.y, delta_drone_obstacle.x) - obstacle.theta;
            ROS_DEBUG("[control]: angle to obstacle: %.3f", angle);
            
            // obstacle.theta + pi/2 is straight ahead for obstacle.

            const bool drone_in_front_of_obstacle = angle >= 0; 
            
            geometry_msgs::Vector3 minimum_delta;
            minimum_delta.z = 0.0; // not used
            if (drone_in_front_of_obstacle){
                ROS_DEBUG_THROTTLE(1, "[control]: drone in front of obstacle");

                minimum_delta.x = clearance_side * std::cos(angle);
                minimum_delta.y = clearance_front/clearance_side * (clearance_side - std::abs(minimum_delta.x));
            }
            else {
                ROS_DEBUG_THROTTLE(1,"[control]: drone behind obstacle");

                minimum_delta.x = clearance_side * std::cos(angle);
                minimum_delta.y = clearance_back * std::sin(angle);
            } 

            const float minimum_distance = std::sqrt(std::pow(minimum_delta.x, 2) + std::pow(minimum_delta.y, 2));
            if (distance_to_obstacle < minimum_distance) {
                // need to avoid obstacle
                setpoint_modified = true;
                setpoint->position.x = obstacle.x + minimum_delta.x;
                setpoint->position.y = obstacle.y + minimum_delta.y;

                ROS_DEBUG_THROTTLE(1, "[control]: obstacle avoidance setpoint: %.2f, %.2f, %.2f", 
                        setpoint->position.x, setpoint->position.y, setpoint->position.z);
            }
        }
    }

    //Return true if setpoint has been modified.
    return setpoint_modified;
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

std::shared_ptr<control::ObstacleAvoidance> control::ObstacleAvoidance::getSharedInstancePtr() {
    //Create new instance if there isn't one
    if(instance_p_ == nullptr) {
        instance_p_ = std::shared_ptr<ObstacleAvoidance>(new ObstacleAvoidance);
    }
    return instance_p_;
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

control::ObstacleAvoidance::ObstacleAvoidance(){
    ros::NodeHandle n("~");
    auto getFloatParam = [&](const std::string& name, float& var) {
        if(!n.getParam(name, var)) {
            ROS_WARN("[Obstacle avoidance] Load param failed: %s, using %f", name.c_str(), var);
        }
    };


    getFloatParam("obstacle_clearance_side", clearance_side);
    getFloatParam("obstacle_clearance_front", clearance_front);
    getFloatParam("obstacle_clearance_back", clearance_back);
    if(!n.getParam("obstacle_clearance_max", clearance_max)) {
        ROS_INFO("[Obstacle avoidance] Load param \"obstacle_clearance_max\" not supplied, computing from others");
        clearance_max = std::max({clearance_side, clearance_back, clearance_front});
    }

}

