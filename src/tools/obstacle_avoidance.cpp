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
geometry_msgs::Vector3 calcMinimumVector(const float drone_angle_to_obstacle, const float drone_direction, const float drone_speed_ratio,
                                              const float clearance_min, const float clearance_max){

    geometry_msgs::Vector3 minimum_vector;

    const auto clearance_front = clearance_min + drone_speed_ratio*(clearance_max - clearance_min);

    if (drone_speed_ratio > 0.1){
        const auto worst_direction = PI + drone_direction;
        const auto angle = angleWrapper(drone_angle_to_obstacle - worst_direction);

        minimum_vector.x = clearance_front * std::cos(angle);
        minimum_vector.y = clearance_min * std::sin(angle);
        minimum_vector = rotateXY(minimum_vector, worst_direction);
        
    } else {
        minimum_vector.x = clearance_min * std::cos(drone_angle_to_obstacle);
        minimum_vector.y = clearance_min * std::sin(drone_angle_to_obstacle);
    }
    
    minimum_vector.z = 0.0f;
    
    return minimum_vector;
}

// V1/2/3 are vector types with x,y,z
// SP is a setpoint supporting SP sp; sp->position.x/y/z
template<typename SP, typename V1, typename V2, typename V3> 
bool checkAndAvoidSingleObstacle(SP* setpoint_p, const V1& drone_position, const V2& drone_velocity, const V3& obstacle_global_position){
    using control::Config;

    const auto drone_distance_to_obstacle = calcDistanceToObstacle(drone_position, obstacle_global_position);
    const auto setpoint_distance_to_obstacle = calcDistanceToObstacle(setpoint_p->position, obstacle_global_position);

    const auto drone_direction = std::atan2(drone_velocity.y, drone_velocity.x);
    const auto drone_speed = std::sqrt(std::pow(drone_velocity.x,2) + std::pow(drone_velocity.y,2));
    const float drone_speed_ratio = std::min((float)drone_speed/1.0f, 1.f); // 1.0 m/s is assumed drone max speed

    bool setpoint_modified = false;
    if (drone_distance_to_obstacle < Config::obstacle_clearance_checkradius){
        // perform obstacle avoidance
        const auto drone_angle_to_obstacle = calcAngleToObstacle(drone_position, obstacle_global_position);
        const auto setpoint_angle_to_obstacle = calcAngleToObstacle(setpoint_p->position, obstacle_global_position);

        // True if setpoint is within a 120 deg cone away from the obstacle
        // TODO: turn this angle into a param once complete
        const bool setpoint_reachable = fabs(fmod(drone_angle_to_obstacle - setpoint_angle_to_obstacle, 2*PI)) < 3*PI/8;

        const auto minimum_vector = calcMinimumVector(drone_angle_to_obstacle, drone_direction, drone_speed_ratio, Config::obstacle_clearance_min, Config::obstacle_clearance_max);

        const auto minimum_distance = std::sqrt(std::pow(minimum_vector.x, 2) + std::pow(minimum_vector.y, 2));
        if (drone_distance_to_obstacle < minimum_distance){
            if (setpoint_reachable
                    && setpoint_distance_to_obstacle > drone_distance_to_obstacle
                    && setpoint_distance_to_obstacle > minimum_distance){
                // no action, maybe logging?
                ROS_DEBUG_THROTTLE(1, "[obstacle avoidance]: Prefer setpoint over avoid algorithm");
            }
            else {
                if (setpoint_modified){
                    // TODO: find out a better solution to this problem
                    ROS_WARN_THROTTLE(1, "[obstacle avoidance]: Two obstacles in range, go up");
                    setpoint_p->position.z = Config::safe_hover_altitude; // hover taller than all obstacles :)
                }
                // need to avoid obstacle
                setpoint_modified = true;

                setpoint_p->position.x = obstacle_global_position.x + minimum_vector.x;
                setpoint_p->position.y = obstacle_global_position.y + minimum_vector.y;
                if (setpoint_p->position.z < Config::min_in_air_alt){
                    setpoint_p->position.z = Config::min_in_air_alt;
                }
            }
        }
    }
    
    return setpoint_modified;
}

bool control::ObstacleAvoidance::doObstacleAvoidance(mavros_msgs::PositionTarget* setpoint_p) {

    bool setpoint_modified{false};

    const auto obstacles = control::ObstacleStateHandler::getCurrentObstacles();
    const auto drone_pose = control::DroneHandler::getCurrentLocalPose();
    const auto drone_velocity = control::DroneHandler::getCurrentTwist().twist.linear;

    ascend_msgs::PolygonArray zone_msg;

    const bool obstacles_valid = control::ObstacleStateHandler::isInstanceReady();

    for (int i = 0; i < obstacles.count && obstacles_valid; i++){
        
        if (checkAndAvoidSingleObstacle(setpoint_p, drone_pose.pose.position, drone_velocity, obstacles.global_robot_position[i])){
            setpoint_modified = true;
        }
    
        // generate points for zone_pub_
        constexpr int N_points{12};
        geometry_msgs::Polygon polygon;
        for (int j = 0; j < N_points; j++){
            const float angle = static_cast<float>(j)*2.f*PI/static_cast<float>(N_points);
            const auto drone_direction = std::atan2(drone_velocity.y, drone_velocity.x);
            const auto drone_speed = std::sqrt(std::pow(drone_velocity.x,2) + std::pow(drone_velocity.y,2));
            const float drone_speed_ratio = std::min((float)drone_speed/2.5f, 1.f); // 2.5 m/s is assumed drone max speed

            const auto local_pos = calcMinimumVector(angle, drone_direction, drone_speed_ratio, control::Config::obstacle_clearance_min, 2.0*control::Config::obstacle_clearance_min);
            
            geometry_msgs::Point32 global_pos;
            global_pos.x = obstacles.global_robot_position[i].x + local_pos.x;
            global_pos.y = obstacles.global_robot_position[i].y + local_pos.y;
            global_pos.z = obstacles.global_robot_position[i].z + local_pos.z;

            polygon.points.push_back(global_pos);
        }
        
        zone_msg.polygons.push_back(polygon);
    }

    zone_pub_.publish(zone_msg);

    return setpoint_modified;
}


/// Update motion of obstacles around a given point
// V1/2: vector type with x,y,z members
template<typename V1, typename V2> 
void simulateObstacleMotion(V1& obstacle_pos, const V2& center, const float time_step) {
    constexpr auto obstacle_vel{0.33f};
    const auto radius = std::sqrt(std::pow(obstacle_pos.x - center.x, 2) + std::pow(obstacle_pos.y - center.y, 2));
    const auto angular_velocity = obstacle_vel/radius;

    // diff eq for circular motion around a center
    const auto vel_x = -angular_velocity*(obstacle_pos.y - center.y);
    const auto vel_y =  angular_velocity*(obstacle_pos.x - center.x);

    obstacle_pos.x += time_step * vel_x;
    obstacle_pos.y += time_step * vel_y;
}

/// Take in a point and return time representing how long that point is safe up to
template<typename P> // P is point (x/y/x)
float areaSafePrediction(const P& checkpoint, const float sim_forward_time) {
    const auto drone_pose = control::DroneHandler::getCurrentLocalPose();
    const auto drone_velocity = control::DroneHandler::getCurrentTwist().twist.linear;
    auto obstacles = control::ObstacleStateHandler::getCurrentObstacles();
    
    const bool obstacles_valid = control::ObstacleStateHandler::isInstanceReady();
    if (!obstacles_valid){
        ROS_WARN_THROTTLE(1, "[obstacle_avoidance]: ObstacleStateHandler not ready"); 
    }

    // emulate relevant parts of mavros_msgs::PositionTarget
    struct Vector3 {
        double x,y,z;
    };
    struct Setpoint_t {
        Vector3 position;
    };
    Setpoint_t setpoint;
    setpoint.position.x = checkpoint.x; setpoint.position.y = checkpoint.y; setpoint.position.z = checkpoint.z;

    const auto dt{0.1f};
    geometry_msgs::Point center;
    center.x = 10.f;
    center.y = 10.f;

    for (auto t{0.f}; t < sim_forward_time; t += dt) {
        for (auto& obstacle_pos : obstacles.global_robot_position){
            // move obstacle forward
            simulateObstacleMotion(obstacle_pos, center, dt);

            // check updated obstacle against the setpoint
            if (checkAndAvoidSingleObstacle(&setpoint, setpoint.position, drone_velocity, obstacle_pos)){
                return t;
            }
        }

        // simulate drone motion here?
        // ...
    }
    
    return sim_forward_time; // maybe return something larger
}

bool control::ObstacleAvoidance::predictSafetyOfPoint(const geometry_msgs::Point& point){
    const bool safe = areaSafePrediction(point, 30.f) >= 30.f;

    return safe;
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

    const mavros_msgs::PositionTarget setpoint_unchanged = setpoint; 

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

