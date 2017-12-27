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

    //Singleton pattern
    static std::unique_ptr<ObstacleAvoidance> instance_p_;

    /**Set of callback method pointers registered by states
     * Pointers are used to allow comparison as std::function can't be compared
     */
    std::set< std::shared_ptr< std::function<void() > > > on_modified_cb_set_;

    ///Notify states the setpoint has been changed
    void onModified();

    ///Runs obstacle avoidance, and modifies setpoint (and notifies)
    bool doObstacleAvoidance(mavros_msgs::PositionTarget* setpoint);

    ///Private constructor
    ObstacleAvoidance() = default;
public:
    ///Deleted copy constructor
    ObstacleAvoidance(const ObstacleAvoidance&) = delete;
    ///Add new callback
    void registerOnModifiedCBPtr(const std::shared_ptr<std::function<void()> >& cb_p) { on_modified_cb_set_.insert(cb_p); }
    ///Remove a callback
    void removeOnModifiedCBPtr(const std::shared_ptr<std::function<void()> >& cb_p);
    ///Modifies setpoint if obstacle is too close
    mavros_msgs::PositionTarget run(mavros_msgs::PositionTarget setpoint);
    ///Returns shared_ptr to one instance
    static ObstacleAvoidance* getSharedInstancePtr();
    ///Returns true when obstacle avoidance is running
    bool isReady() { return true; }

};
}

#endif
