#include "control/tools/obstacle_avoidance.hpp"
#include "control/tools/obstacle_state_handler.hpp"
#include "control/tools/drone_handler.hpp"
#include "control/tools/config.hpp"

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

inline float angleWrapper(float angle){
    return angle - 2*PI*floor(angle/(2*PI));
}

bool control::ObstacleAvoidance::doObstacleAvoidance(mavros_msgs::PositionTarget* setpoint) {
    using control::Config; 

    bool setpoint_modified = false;
    
    const std::vector<ascend_msgs::GRState> obstacles = control::ObstacleStateHandler::getCurrentObstacles();
    const geometry_msgs::PoseStamped drone_pose = control::DroneHandler::getCurrentPose();

    for (const auto& obstacle : obstacles){

        geometry_msgs::Vector3 delta_drone_obstacle;
        delta_drone_obstacle.x = drone_pose.pose.position.x - obstacle.x; 
        delta_drone_obstacle.y = drone_pose.pose.position.y - obstacle.y;
        delta_drone_obstacle.z = drone_pose.pose.position.z; 
        const float distance_to_obstacle = std::sqrt(std::pow(delta_drone_obstacle.x, 2) + std::pow(delta_drone_obstacle.y, 2));

        if (distance_to_obstacle < Config::obstacle_clearance_checkradius){
            // perform obstacle avoidance
            const float angle_to_obstacle = angleWrapper(std::atan2(delta_drone_obstacle.y, delta_drone_obstacle.x) - obstacle.theta);
//            ROS_INFO_THROTTLE(1, "[control] angle: %.3f", 
//                    /*obstacle.theta, std::atan2(delta_drone_obstacle.y, delta_drone_obstacle.x),*/ angle);
            
            // obstacle.theta + pi/2 is straight ahead of obstacle.

            const bool drone_in_front_of_obstacle = angle_to_obstacle <= PI; 
            
            geometry_msgs::Vector3 minimum_delta;
            minimum_delta.z = 0.0; // not used
            if (drone_in_front_of_obstacle){
                //ROS_INFO_THROTTLE(0.2, "[control]: %.2f -> drone in front of obstacle", angle_to_obstacle);

                minimum_delta.x = Config::obstacle_clearance_side * std::cos(angle_to_obstacle);
                minimum_delta.y = Config::obstacle_clearance_front/Config::obstacle_clearance_side * (Config::obstacle_clearance_side - std::abs(minimum_delta.x));
            }
            else {
                //ROS_INFO_THROTTLE(0.2, "[control]: %.2f -> drone behind obstacle", angle_to_obstacle);

                minimum_delta.x = Config::obstacle_clearance_side * std::cos(angle_to_obstacle);
                minimum_delta.y = Config::obstacle_clearance_back * std::sin(angle_to_obstacle);
            } 

            const float minimum_distance = std::sqrt(std::pow(minimum_delta.x, 2) + std::pow(minimum_delta.y, 2));
            ROS_INFO_THROTTLE(1, "distances: %.2f\t[%.2f]", distance_to_obstacle, minimum_distance);
            if (distance_to_obstacle < minimum_distance) {
                // need to avoid obstacle
                setpoint_modified = true;
                setpoint->position.x = obstacle.x + minimum_delta.x;
                setpoint->position.y = obstacle.y + minimum_delta.y;
            }
        }
    }
    
    // debug helpers
    if (setpoint_modified){
        ROS_INFO_THROTTLE(1 , "setpoint changed");
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

