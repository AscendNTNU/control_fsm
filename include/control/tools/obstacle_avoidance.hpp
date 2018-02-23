#ifndef OBSTACLE_AVOIDANCE_HPP
#define OBSTACLE_AVOIDANCE_HPP
#include <set>
#include <functional>
#include <mavros_msgs/PositionTarget.h>
#include <memory>

namespace control {

///Low level obstacle avoidance
class ObstacleAvoidance;
class ObstacleAvoidance {
private:
    /**Set of callback method pointers registered by states
     * Pointers are used to allow comparison as std::function can't be compared
     */
    std::set< std::shared_ptr< std::function<void() > > > on_modified_cb_set_;

    ///Notify states the setpoint has been changed
    void onModified();

    ///Runs obstacle avoidance, and modifies setpoint (and notifies)
    virtual bool doObstacleAvoidance(mavros_msgs::PositionTarget* setpoint);

public:
    ///Default constructor
    ObstacleAvoidance() = default;
    ///Default copy constructor
    ObstacleAvoidance(const ObstacleAvoidance&) = default;
    ///Add new callback
    void registerOnModifiedCBPtr(const std::shared_ptr<std::function<void()> >& cb_p) { on_modified_cb_set_.insert(cb_p); }
    ///Remove a callback
    void removeOnModifiedCBPtr(const std::shared_ptr<std::function<void()> >& cb_p);
    ///Modifies setpoint if obstacle is too close
    mavros_msgs::PositionTarget run(mavros_msgs::PositionTarget setpoint);
    ///Returns true when obstacle avoidance is running
    bool isReady() { return true; }

};

// Only do this for unit testing
#ifdef CONTROL_FSM_UNIT_TEST
#include <geometry_msgs/Vector3.h>

float angleWrapper(const float angle);

template<typename T, typename K>
inline geometry_msgs::Vector3 calcVectorBetweenPoints(const T& point_A, const K& point_B);

template<typename T, typename K>
inline float calcDistanceToObstacle(const T& point, const K& obstacle_position);

template<typename T, typename K>
inline float calcAngleToObstacle(const T& point, const K& obstacle_position, const float obstacle_direction);

template<typename T>
inline geometry_msgs::Vector3 rotateXY(const T& point, const float angle);
#endif
}

#endif
