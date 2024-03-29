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
class Config;
class Config {
private:
    ///Node handler
    ros::NodeHandle nh_;
    ///Reload service
    ros::ServiceServer reload_config_service;
    static std::set<std::string> missing_param_set_;
    ///Shared instance ptr
    static std::unique_ptr<Config> shared_instance_p_;
    ///Constructor
    Config();


public:
    /// \fsmparam Are we close enough to the target?
    static double dest_reached_margin;
    /// \fsmparam Default hover altitude in case of blind hover
    static double blind_hover_alt;
    /// \fsmparam Takeoff altitude
    static double takeoff_altitude;
    /// \fsmparam Minmum allowed position hold altitude
    static double min_in_air_alt;
    /// \fsmparam Are we close enough to target altitude
    static double altitude_reached_margin;
    /// \fsmparam Are we close enough to setpoint
    static double setpoint_reached_margin;
    /// \fsmparam Is yaw close enough?
    static double yaw_reached_margin;
    /// \fsmparam What is our cruising altitude when landing at point xy?
    static double land_xy_goto_alt;
    //// \fsmparam Margin to determine when velocity is reached
    static double velocity_reached_margin;
    /// \fsmparam Search altitude
    static double gb_search_altitude;
    /// \fsmparam Topic to publish FSM error msg
    static std::string fsm_error_topic;
    /// \fsmparam Topic to publish FSM warn msg
    static std::string fsm_warn_topic;
    /// \fsmparam Topic to publish FSM info msg
    static std::string fsm_info_topic;
    /// \fsmparam Topic to publish FSM state changed
    static std::string fsm_state_changed_topic;
    /// \fsmparam Topic for recieving local position from FC
    static std::string mavros_local_pos_topic;
    /// \fsmparam Topic for recieving distance from FC 
    static std::string mavros_distance_sensor_topic;
    /// \fsmparam Topic for recieving local velocity from FC
    static std::string mavros_local_vel_topic;
    /// \fsmparam Topic for recieving current state from FC
    static std::string mavros_state_changed_topic;
    /// \fsmparam Topic for recieving current state from land detector
    static std::string land_detector_topic;
    /// \fsmparam Which type of land detector to use
    static std::string land_detector_type;
    /// \fsmparam Topic for recieving obstacle positions
    static std::string obstacle_state_topic;
    /// \fsmparam Topic for recieving debug service requests
    static std::string debug_server_topic;
    /// \fsmparam Topic for action server
    static std::string action_server_topic;
    /// \fsmparam Buffer size used for FSMError, FSMWarn etc
    static int fsm_status_buffer_size;
    /// \fsmparam Time gotostate waits before transitioning
    static double go_to_hold_dest_time;
    /// \fsmparam Distance used to determine if goto yaw should be calculated
    static double no_yaw_correct_dist;
    /// \fsmparam Altitude where the drone is safe from all obstacles
    static double safe_hover_altitude;
    /// \fsmparam Drone safezone
    static double obstacle_too_close_dist;
    /// \fsmparam Clearance osbtacle avoidance 
    static double obstacle_clearance_max;
    static double obstacle_clearance_min;
    /// \fsmparam Radius in which obstacle collisions will be checked for
    static double obstacle_clearance_checkradius;
    /// \fsmparam Topic to listen for info about obstacles
    static std::string lidar_topic;
    /// \fsmparam Finished drone will require all datastreams to be available
    static bool require_all_data_streams;
    /// \fsmparam Makes it possible to disable the use of obstacle detection features
    static bool require_obstacle_detection;
    /// \fsmparam When is data considered old?
    static double valid_data_timeout;
    /// \fsmparam Frame id for global frame
    static std::string global_frame_id;
    /// \fsmparam Frame id for local frame
    static std::string local_frame_id;
    /// \fsmparam Use transform for global position
    static bool use_global_transforms;
    /// \fsmparam Restrict drone position to within arena
    static bool restrict_arena_boundaries;
    /// \fsmparam Lowest allowed x global position
    static double arena_lowest_x;
    /// \fsmparam Lowest allowed y global position
    static double arena_lowest_y;
    /// \fsmparam Highest allowed x global position
    static double arena_highest_x;
    /// \fsmparam Highest allowed y global position
    static double arena_highest_y;
    /// \fsmparam Require use of distance sensor to allow transition to idle
    static bool require_distance_sensor;

    /**Load paramaters
     * @throw control::ROSNotInitializedException
     */
    static void loadParams();
    
    ///Returns set of unloaded params
    static const std::set<std::string>& getMissingParamSet() { return missing_param_set_; }
};
}

#endif
