#include "control/tools/obstacle_avoidance.hpp"
#include "control/tools/obstacle_state_handler.hpp"
#include "control/tools/drone_handler.hpp"
#include "control/tools/config.hpp"

#include <ascend_msgs/GRStateArray.h>
#include <ascend_msgs/GRState.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h> // not used?

#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

constexpr float PI{3.14159265359f};

// wrap any angle to range [0, 2pi)
inline float angleWrapper(const float angle){
    return angle - 2*PI*floor(angle/(2*PI));
}

template<typename T, typename K>
inline geometry_msgs::Vector3 calcVectorBetweenPoints(const T& point_A, const K& point_B){
    geometry_msgs::Vector3 vector_diff;
    vector_diff.x = point_A.x - point_B.x; 
    vector_diff.y = point_A.y - point_B.y;
    vector_diff.z = point_A.z - point_B.z;

    return vector_diff;
}

template<typename T, typename K>
inline float calcDistanceToObstacle(const T& point, const K& obstacle_position){
    const auto vector_to_obstacle = calcVectorBetweenPoints(point, obstacle_position);
    const float distance_to_obstacle = std::sqrt(std::pow(vector_to_obstacle.x, 2) + std::pow(vector_to_obstacle.y, 2));

    return distance_to_obstacle;
}



template<typename T, typename K>
inline float calcAngleToObstacle(const T& point, const K& obstacle_position, const float obstacle_direction){
    geometry_msgs::Vector3 delta_drone_obstacle;
    delta_drone_obstacle.x = point.x - obstacle_position.x; 
    delta_drone_obstacle.y = point.y - obstacle_position.y;
    delta_drone_obstacle.z = point.z; 

    const float angle_to_obstacle = angleWrapper(-PI/2 + std::atan2(delta_drone_obstacle.y, delta_drone_obstacle.x) - obstacle_direction);
    
    return angle_to_obstacle;
}

template<typename T>
inline geometry_msgs::Vector3 rotateXY(const T& point, const float angle){
    // Apply 2d transformation matrix
    T new_point;
    new_point.x = point.x * std::cos(angle) - point.y * std::sin(angle);
    new_point.y = point.x * std::sin(angle) + point.y * std::cos(angle);
    new_point.z = point.z;

    return new_point;
}

/// Return a vector (x,y,z) which corresponds to the point closest to the obstacle
/// which is allowed, referenced from the obstacles coordinate system. x-axis is positive 
/// direction of motion. Obstacle driving in with obstacle_direction=0 would be driving in x-direction
inline geometry_msgs::Vector3 avoidZone(const float obstacle_direction, 
        const float front_clearance, const float back_clearance, const float side_clearance){

    const auto angle_positive = angleWrapper(obstacle_direction);
    const bool drone_in_front_of_obstacle = angle_positive <=  PI/2 || angle_positive >= 3*PI/2;

    geometry_msgs::Vector3 minimum_vector;
    minimum_vector.z = 0.0f; 

    minimum_vector.y = side_clearance * std::sin(obstacle_direction);
    if (drone_in_front_of_obstacle){
        // Triangle shape
        //minimum_vector.x = (front_clearance/side_clearance)*(side_clearance - std::fabs(minimum_vector.y));
	
	// Ellipse
	minimum_vector.x = front_clearance * std::cos(obstacle_direction);
    }
    else {
        // Ellipse
        minimum_vector.x = back_clearance * std::cos(obstacle_direction);
    }

    return minimum_vector;
}

bool control::ObstacleAvoidance::doObstacleAvoidance(mavros_msgs::PositionTarget* setpoint) {
    using control::Config; 
    bool setpoint_modified{false};

    const auto obstacles = control::ObstacleStateHandler::getCurrentObstacles(); // Format of isn't finalized yet
    const geometry_msgs::PoseStamped drone_pose = control::DroneHandler::getCurrentPose();


    const auto drone_vel = control::DroneHandler::getCurrentTwist().twist.linear;
    const auto drone_speed = std::sqrt(std::pow(drone_vel.x,2) + std::pow(drone_vel.y,2));
    const float drone_speed_ratio = std::sqrt(drone_speed/3.f);

    for (int i = 0; i < obstacles.count; i++){
        const auto obstacle_position = obstacles.global_robot_position[i];
        const float obstacle_direction = obstacles.direction[i];
        const auto drone_distance_to_obstacle = calcDistanceToObstacle(drone_pose.pose.position, obstacle_position);
        const auto setpoint_distance_to_obstacle = calcDistanceToObstacle(setpoint->position, obstacle_position);

        if (drone_distance_to_obstacle < Config::obstacle_clearance_checkradius){ 
            // perform obstacle avoidance
            const auto drone_angle_to_obstacle = calcAngleToObstacle(drone_pose.pose.position, obstacle_position, obstacle_direction);
            const auto setpoint_angle_to_obstacle = calcAngleToObstacle(setpoint->position, obstacle_position, obstacle_direction);

            // True if setpoint is within a 120 deg cone away from the obstacle
            // TODO: turn this angle into a param once complete
            const bool setpoint_reachable = 
                (drone_angle_to_obstacle - 3*PI/8) < setpoint_angle_to_obstacle &&
                setpoint_angle_to_obstacle < (drone_angle_to_obstacle + 3*PI/8);


	    // magic numbers are minimum clearance no matter what. Turn into parameters and include
	    // this logic in the avoidZone function
            const auto clearance_front = std::max((float)Config::obstacle_clearance_front*drone_speed_ratio, 1.f);
            const auto clearance_back  = std::max((float)Config::obstacle_clearance_back*drone_speed_ratio, 0.7f);
            const auto clearance_side  = std::max((float)Config::obstacle_clearance_side*drone_speed_ratio, 0.7f);
            geometry_msgs::Vector3 minimum_vector = avoidZone(drone_angle_to_obstacle,
                    clearance_front, clearance_back, clearance_side);

            // Rotate to global coordinate system
            const auto minimum_distance = std::sqrt(std::pow(minimum_vector.x, 2) + std::pow(minimum_vector.y, 2));
	//    ROS_INFO_THROTTLE(1, "Minimum vector: %.3f\t%.3f\t(%.3f)\t(%.3f)", minimum_vector.x, minimum_vector.y, 
	//		    minimum_distance, drone_angle_to_obstacle);

            minimum_vector = rotateXY(minimum_vector, -obstacle_direction);

	    //ROS_INFO("Minimum pos: %.3f\t%.3f", obstacle_position.x + minimum_vector.x, obstacle_position.y + minimum_vector.y);


	    if (drone_distance_to_obstacle < minimum_distance){
                if (setpoint_reachable
                        && setpoint_distance_to_obstacle > drone_distance_to_obstacle 
                        && setpoint_distance_to_obstacle > minimum_distance){
                    // no action, maybe logging?
                    ROS_INFO_THROTTLE(1, "Going to setpoint instead");
                }
                else {
                    const geometry_msgs::Vector3 drone_delta_to_obstacle 
                        = calcVectorBetweenPoints(drone_pose.pose.position, obstacle_position);

                    if (setpoint_modified){
                        // TODO: find out a better solution to this problem
                        ROS_WARN("[obstacle avoidance]: Two obstacles in range, undefined behaviour!");
                        setpoint->position.z = Config::safe_hover_altitude; // hover taller than all obstacles :)
                    }
                    // need to avoid obstacle
                    setpoint_modified = true;
                    setpoint->position.x = obstacle_position.x + minimum_vector.x;
                    setpoint->position.y = obstacle_position.y + minimum_vector.y;
                    if (setpoint->position.z < Config::min_in_air_alt){
                        setpoint->position.z = Config::min_in_air_alt;
                    }

                    ROS_INFO_THROTTLE(0.25,"Avoiding: %.3f -> %.3f", drone_distance_to_obstacle, minimum_distance);
                }
	    }
        }

        if (setpoint_modified){
            const auto new_distance_to_obstacle = calcDistanceToObstacle(setpoint->position, obstacle_position);
            ROS_INFO_THROTTLE(1, "Distance improvement %.3f to %.3f", drone_distance_to_obstacle, new_distance_to_obstacle);
        }
    }

    if (!setpoint_modified){
        ROS_INFO_THROTTLE(1,"[obstacle avoidance] no dangerous obstacles");
    }

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

void control::ObstacleAvoidance::removeOnModifiedCBPtr(const std::shared_ptr<std::function<void()> >& cb_p) {
    const auto it = on_modified_cb_set_.find(cb_p);
    if (it != on_modified_cb_set_.cend()){
        on_modified_cb_set_.erase(it);
    }
}

