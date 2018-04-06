#include <ros/ros.h>

#include "control/tools/obstacle_avoidance.hpp"
#include "control/tools/obstacle_math.hpp"
#include "control/tools/obstacle_state_handler.hpp"
#include "control/tools/drone_handler.hpp"
#include "control/tools/config.hpp"

#include <ascend_msgs/PolygonArray.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

using obstacle_math::PI;
using obstacle_math::rotateXY;
using obstacle_math::angleWrapper;
using obstacle_math::vectorSum;
using obstacle_math::vectorDifference;
using obstacle_math::calcAngleToObstacle;
using obstacle_math::calcDistanceToObstacle;

/// Return a vector (x,y,z) which corresponds to the point closest to the obstacle
/// which is allowed, referenced from the obstacles coordinate system. x-axis is positive
/// direction of motion. Obstacle driving in with obstacle_direction=0 would be driving in x-direction
inline geometry_msgs::Vector3 avoidZone(const float drone_angle_to_obstacle, const float clearance){

    geometry_msgs::Vector3 minimum_vector;
    minimum_vector.x = clearance * std::cos(drone_angle_to_obstacle);
    minimum_vector.y = clearance * std::sin(drone_angle_to_obstacle);
    minimum_vector.z = 0.0f;

    return minimum_vector;
}

bool control::ObstacleAvoidance::doObstacleAvoidance(mavros_msgs::PositionTarget* setpoint) {
    using control::Config;

    bool setpoint_modified{false};

    const auto obstacles = control::ObstacleStateHandler::getCurrentObstacles();
    const auto drone_pose = control::DroneHandler::getCurrentLocalPose();

    //TODO Verify obstacles is local!

    const auto drone_vel = control::DroneHandler::getCurrentTwist().twist.linear;
    const auto drone_speed = std::sqrt(std::pow(drone_vel.x,2) + std::pow(drone_vel.y,2));
    const float drone_speed_ratio = std::min((float)drone_speed/2.5f, 1.f); // 2.5 m/s is assumed drone max speed

    ascend_msgs::PolygonArray zone_msg;

    bool obstacles_valid = control::ObstacleStateHandler::isInstanceReady();

    for (int i = 0; i < obstacles.count && obstacles_valid; i++){
        const auto& obstacle_global_position = obstacles.global_robot_position[i];
        const auto drone_distance_to_obstacle = calcDistanceToObstacle(drone_pose.pose.position, obstacle_global_position);
        const auto setpoint_distance_to_obstacle = calcDistanceToObstacle(setpoint->position, obstacle_global_position);

        if (drone_distance_to_obstacle < Config::obstacle_clearance_checkradius){
            // perform obstacle avoidance
            const auto drone_angle_to_obstacle = calcAngleToObstacle(drone_pose.pose.position, obstacle_global_position);
            const auto setpoint_angle_to_obstacle = calcAngleToObstacle(setpoint->position, obstacle_global_position);

            // True if setpoint is within a 120 deg cone away from the obstacle
            // TODO: turn this angle into a param once complete
            const bool setpoint_reachable = fabs(fmod(drone_angle_to_obstacle - setpoint_angle_to_obstacle, 2*PI)) < 3*PI/8;

            const auto clearance = std::max(Config::obstacle_clearance_max*drone_speed_ratio, Config::obstacle_clearance_min);
            geometry_msgs::Vector3 minimum_vector = avoidZone(drone_angle_to_obstacle, clearance);

            // Rotate to global coordinate system
            const auto minimum_distance = std::sqrt(std::pow(minimum_vector.x, 2) + std::pow(minimum_vector.y, 2));

            //minimum_vector = rotateXY(minimum_vector, obstacle_direction);

            //ROS_INFO("Minimum pos: %.3f\t%.3f", obstacle_global_position.x + minimum_vector.x, obstacle_global_position.y + minimum_vector.y);

            // generate points for zone_pub_
            constexpr int N_points{4};
            geometry_msgs::Polygon polygon;
            for (int i = 0; i < N_points; i++){
                const float angle = static_cast<float>(i)*2.f*PI/N_points;
                auto local_pos = avoidZone(angle, clearance);
                local_pos = vectorSum(local_pos, obstacle_global_position);

                geometry_msgs::Point32 global_pos;
                global_pos.x = obstacle_global_position.x + local_pos.x;
                global_pos.y = obstacle_global_position.y + local_pos.y;
                global_pos.z = obstacle_global_position.z + local_pos.z;
                
                polygon.points.push_back(global_pos);
            }

            zone_msg.polygons.push_back(polygon);

            if (drone_distance_to_obstacle < minimum_distance){
                if (setpoint_reachable
                        && setpoint_distance_to_obstacle > drone_distance_to_obstacle
                        && setpoint_distance_to_obstacle > minimum_distance){
                    // no action, maybe logging?
                    ROS_INFO_THROTTLE(1, "[obstacle avoidance]: Prefer setpoint over avoid algorithm");
                }
                else {
                    if (setpoint_modified){
                        // TODO: find out a better solution to this problem
                        ROS_WARN("[obstacle avoidance]: Two obstacles in range, undefined behaviour!");
                        setpoint->position.z = Config::safe_hover_altitude; // hover taller than all obstacles :)
                    }
                    // need to avoid obstacle
                    setpoint_modified = true;
                    setpoint->position.x = obstacle_global_position.x + minimum_vector.x;
                    setpoint->position.y = obstacle_global_position.y + minimum_vector.y;
                    if (setpoint->position.z < Config::min_in_air_alt){
                        setpoint->position.z = Config::min_in_air_alt;
                    }
                }
	    }
        }

        if (setpoint_modified){
            //const auto new_distance_to_obstacle = calcDistanceToObstacle(setpoint->position, obstacle_global_position);
            //ROS_INFO_THROTTLE(1, "Distance improvement %.3f to %.3f", drone_distance_to_obstacle, new_distance_to_obstacle);
        }
    }

    zone_pub_.publish(zone_msg);

    return setpoint_modified;
}

void control::ObstacleAvoidance::onModified() {
    //Run all callbacks
    for(auto& cb_p : on_modified_cb_set_ ) {
        (*cb_p)();
    }
}

void control::ObstacleAvoidance::onWarn() {
    //Run all callbacks
    for(auto& cb_p : on_warn_cb_set_ ) {
        (*cb_p)();
    }
}

mavros_msgs::PositionTarget control::ObstacleAvoidance::run(mavros_msgs::PositionTarget setpoint) {
    mavros_msgs::PositionTarget setpoint_unchanged = setpoint; 

    const bool setpoint_modified = doObstacleAvoidance(&setpoint);

    if (setpoint_modified){
        //Notify states
        onWarn();
        if (has_setpoint_responsibility_){
            onModified();
            return setpoint;
        } 
    }

    return setpoint_unchanged;
}

void control::ObstacleAvoidance::removeOnModifiedCBPtr(const std::shared_ptr<std::function<void()> >& cb_p) {
    const auto it = on_modified_cb_set_.find(cb_p);
    if (it != on_modified_cb_set_.cend()){
        on_modified_cb_set_.erase(it);
    }
}

void control::ObstacleAvoidance::removeOnWarnCBPtr(const std::shared_ptr<std::function<void()> >& cb_p) {
    const auto it = on_warn_cb_set_.find(cb_p);
    if (it != on_warn_cb_set_.cend()) {
        on_warn_cb_set_.erase(it);
    }
}


control::ObstacleAvoidance::ObstacleAvoidance(){
    zone_pub_ = nh_.advertise<ascend_msgs::PolygonArray>("/control/obstacle_avoidance/polygons", 10);    
}

