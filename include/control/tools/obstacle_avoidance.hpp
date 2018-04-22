#ifndef OBSTACLE_AVOIDANCE_HPP
#define OBSTACLE_AVOIDANCE_HPP
#include <set>
#include <functional>
#include <mavros_msgs/PositionTarget.h>
#include <memory>
#include <ros/ros.h>

namespace control {

///Low level obstacle avoidance
class ObstacleAvoidance;
class ObstacleAvoidance {
private:
    /**Set of modified callback method pointers registered by states
     * Pointers are used to allow comparison as std::function can't be compared
     */
    std::set< std::shared_ptr< std::function<void() > > > on_modified_cb_set_;
    
    /**Set of warn callback method pointers registered by states
     * Pointers are used to allow comparison as std::function can't be compared
     */
    std::set< std::shared_ptr< std::function<void() > > > on_warn_cb_set_;

    ///Notify states the setpoint has been changed
    void onModified();

    ///Notify states the setpoint should be changed
    void onWarn();

    ///Runs obstacle avoidance, and modifies setpoint (and notifies)
    virtual bool doObstacleAvoidance(mavros_msgs::PositionTarget* setpoint);

    ///Nodehandle
    ros::NodeHandle nh_;

    ///Publisher for avoid zone information
    ros::Publisher zone_pub_;

    ///Responsibility flag
    bool has_setpoint_responsibility_ = true;

public:
    ///Default constructor
    ObstacleAvoidance();
    ///Default copy constructor
    ObstacleAvoidance(const ObstacleAvoidance&) = default;

    ///Add new on warn callback
    void registerOnWarnCBPtr(const std::shared_ptr<std::function<void()> >& cb_p) { on_warn_cb_set_.insert(cb_p); }
    ///Remove on warn callback
    void removeOnWarnCBPtr(const std::shared_ptr<std::function<void()> >& cb_p);
    ///Add new on modified callback
    void registerOnModifiedCBPtr(const std::shared_ptr<std::function<void()> >& cb_p) { on_modified_cb_set_.insert(cb_p); }
    ///Remove on modified callback
    void removeOnModifiedCBPtr(const std::shared_ptr<std::function<void()> >& cb_p);

    ///Obstacle avoidance has reduced responsibility. 
    ///It won't change setpoints and only calls warning level callbacks
    void relaxResponsibility() { 
	    ROS_INFO("[control]: Low-Level obstacle-avoidance disabled");
	    has_setpoint_responsibility_ = false; 
    }
    ///Obstacle avoidance takes full responsibility over setpoints and calls all callbacks
    void takeResponsibility() { 
	    ROS_INFO("[control]: Low-Level obstacle-avoidance enabled");
	    has_setpoint_responsibility_ = true; 
    }

    ///Modifies setpoint if obstacle is too close
    mavros_msgs::PositionTarget run(mavros_msgs::PositionTarget setpoint);
    ///Returns true when obstacle avoidance is running
    bool isReady() { return true; }
};
}

#endif
