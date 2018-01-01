#ifndef FSM_CONFIG_HPP
#define FSM_CONFIG_HPP
#include <iostream>
#include <ros/ros.h>
namespace control {
class Config {
public:
    ///Are we close enough to the target?
    static double dest_reached_margin;
    ///Default hover altitude in case of blind hover
    static double blind_hover_alt;
    ///Takeoff altitude
    static double takeoff_altitude;
    ///Are we close enough to target altitude
    static double altitude_reached_margin;
    ///Are we close enough to setpoint
    static double setpoint_reached_margin;
    ///Is yaw close enough?
    static double yaw_reached_margin;
    ///Search altitude
    static double gb_search_altitude;
    ///Topic to publish FSM error msg
    static std::string fsm_error_topic;
    ///Topic to publish FSM warn msg
    static std::string fsm_warn_topic;
    ///Topic to publish FSM info msg
    static std::string fsm_info_topic;
    ///Topic to publish FSM state changed
    static std::string fsm_state_changed_topic;
    ///Topic for recieving local position from FC
    static std::string mavros_local_pos_topic;
    ///Topic for recieving current state from FC
    static std::string mavros_state_changed_topic;
    ///Topic for recieving current state from land detector
    static std::string land_detector_topic;
    ///Buffer size used for FSMError, FSMWarn etc
    static int fsm_status_buffer_size;
    ///Time gotostate waits before transitioning
    static double go_to_hold_dest_time;
    ///Distance used to determine if goto yaw should be calculated
    static double no_yaw_correct_dist;
    ///Altitude where the drone is safe from all obstacles
    static double safe_hover_altitude;
    ///Drone safezone
    static double obstacle_too_close_dist;
    ///Topic to listen for info about obstacles
    static std::string lidar_topic;
    /**Load paramaters
     * @throw control::ROSNotInitializedException
     */
    static void loadParams();
    ///Finished drone will require all datastreams to be available
    static bool require_all_data_streams;
    ///Makes it possible to disable the use of obstacle detection features
    static bool require_obstacle_detection;
    ///When is data considered old?
    static double valid_data_timeout;
};
}

#endif
