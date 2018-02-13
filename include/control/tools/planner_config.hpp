#ifndef FSM_CONFIG_HPP
#define FSM_CONFIG_HPP
#include <iostream>
#include <ros/ros.h>
#include <set>

/** @page fsm_params FSM Params
 *  @brief Parameter page
 *
 *  Contains all fsm params
 */


namespace control {
class PlannerConfig;
class PlannerConfig {
private:
    ///Node handler
    ros::NodeHandle nh_;
    ///Reload service
    ros::ServiceServer reload_config_service;
    static std::set<std::string> missing_param_set_;
    ///Shared instance ptr
    static std::unique_ptr<PlannerConfig> shared_instance_p_;
    ///Constructor
    PlannerConfig();


public:
    /// \fsmparam Topic for recieving obstacle positions
    static std::string obstacle_state_topic;

    /**Load paramaters
     * @throw control::ROSNotInitializedException
     */
    static void loadParams();
    
    ///Returns set of unloaded params
    static const std::set<std::string>& getMissingParamSet() { return missing_param_set_; }
};
}

#endif
