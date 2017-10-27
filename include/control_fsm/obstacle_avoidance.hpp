#ifndef OBSTACLE_AVOIDANCE_HPP
#define OBSTACLE_AVOIDANCE_HPP
#include <vector>
#include <functional>
#include <mavros_msgs/PositionTarget.h>
#include <memory>

namespace control {
///Low level obstacle avoidance
class ObstacleAvoidance;
class ObstacleAvoidance {
private:

    //Singleton pattern
    static std::shared_ptr<ObstacleAvoidance> instance_p_;

    ///Queue of callback methods registered by states
    std::vector< std::function<void()> > on_modified_cb_vec_;

    ///Notify states the setpoint has been changed
    void onModified();

    ///Runs obstacle avoidance, and modifies setpoint (and notifies)
    bool doObstacleAvoidance(mavros_msgs::PositionTarget* setpoint);

    ///Private constructor
    ObstacleAvoidance() = default;
    ///Deleted copy constructor
    ObstacleAvoidance(const ObstacleAvoidance&) = delete;

public:
    ///Add new callback
    void registerOnModifiedCB(std::function<void()> cb) { on_modified_cb_vec_.push_back(cb); }
    ///Modifies setpoint if obstacle is too close
    mavros_msgs::PositionTarget run(mavros_msgs::PositionTarget setpoint);
    ///Returns shared_ptr to one instance
    static std::shared_ptr<ObstacleAvoidance> getSharedInstancePtr();
    ///Returns true when obstacle avoidance is running
    bool isReady() { return true; }

};
}

#endif