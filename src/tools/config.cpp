#include "control/tools/config.hpp"
#include <assert.h>
#include <control/exceptions/ros_not_initialized_exception.hpp>

using control::Config;
using control::ROSNotInitializedException;

double Config::dest_reached_margin = 0.3;
double Config::blind_hover_alt = 1.0;
double Config::takeoff_altitude = 1.0;
double Config::altitude_reached_margin = 0.1;
double Config::setpoint_reached_margin = 0.3;
double Config::yaw_reached_margin = 0.02;
double Config::gb_search_altitude = 2.5;
double Config::no_yaw_correct_dist = 0.2;
double Config::valid_data_timeout = 2.0; //2 seconds
std::string Config::fsm_error_topic = "control/fsm/on_error";
std::string Config::fsm_warn_topic = "control/fsm/on_warn";
std::string Config::fsm_info_topic = "control/fsm/on_info";
std::string Config::fsm_state_changed_topic = "control/fsm/state_changed";
std::string Config::mavros_local_pos_topic = "mavros/local_position/pose";
std::string Config::mavros_state_changed_topic = "mavros/state";
std::string Config::land_detector_topic = "/landdetector";
std::string Config::obstacle_state_topic = "/perception_obstacle_states";
int Config::fsm_status_buffer_size = 10;
double Config::go_to_hold_dest_time = 0.5;
double Config::safe_hover_altitude = 2.0;
double Config::obstacle_too_close_dist = 2.0;
std::string Config::lidar_topic = "perception/obstacles/lidar";
bool Config::require_all_data_streams = true;
bool Config::require_obstacle_detection = true;
float Config::obstacle_clearance_side = 2.0f;
float Config::obstacle_clearance_front = 2.0f;
float Config::obstacle_clearance_back = 2.0f;
float Config::obstacle_clearance_checkradius = 2.0f;

void Config::loadParams() {
    if(!ros::isInitialized()) {
        throw ROSNotInitializedException();
    }

    ros::NodeHandle n("~");

    auto getFloatParam = [&](const std::string& name, float& var) {
        if(n.getParam(name, var)) {
            ROS_WARN("[Control FSM] Load param failed: %s, using %f", name.c_str(), var);
        }
    };

    auto getDoubleParam = [&](const std::string& name, double& var) {
        if(!n.getParam(name, var)) {
            ROS_WARN("[Control FSM] Load param failed: %s, using %f", name.c_str(), var);
        }
    };

    auto getStringParam = [&](const std::string& name, std::string& var) {
        if(!n.getParam(name, var)) {
            ROS_WARN("[Control FSM] Load param failed: %s, using %s", name.c_str(), var.c_str());
        }
    };

    auto getIntParam = [&](const std::string& name, int& var) {
        if(!n.getParam(name, var)) {
            ROS_WARN("[Control FSM] Load param failed: %s, using %d", name.c_str(), var);
        }
    };

    auto getBoolParam = [&](const std::string& name, bool& var) {
        if(!n.getParam(name, var)) {
            ROS_WARN("[Control FSM] Load param failed: %s, using %s", name.c_str(), var ? "true" : "false");
        }
    };

    //Global params (used by multiple states)
    getDoubleParam("blind_hover_altitude", blind_hover_alt);
    getDoubleParam("takeoff_altitude", takeoff_altitude);
    getDoubleParam("altitude_reached_margin", altitude_reached_margin);
    getDoubleParam("yaw_reached_margin", yaw_reached_margin);
    getDoubleParam("safe_hover_alt", safe_hover_altitude);
    getDoubleParam("obstacle_too_close_dist", obstacle_too_close_dist);
    getDoubleParam("gb_search_altitude", gb_search_altitude);
    getDoubleParam("message_timeout", valid_data_timeout);
    getBoolParam("require_all_streams", require_all_data_streams);
    getBoolParam("require_obs_detection", require_obstacle_detection);
    //GoTo params
    getDoubleParam("goto_hold_dest_time", go_to_hold_dest_time);
    getDoubleParam("setp_reached_margin", setpoint_reached_margin);
    getDoubleParam("dest_reached_margin", dest_reached_margin);
    getDoubleParam("no_yaw_correct_dist", no_yaw_correct_dist);
    //FSM debug params
    getStringParam("fsm_error_topic", fsm_error_topic);
    getStringParam("fsm_warn_topic", fsm_warn_topic);
    getStringParam("fsm_info_topic", fsm_info_topic);
    getStringParam("fsm_state_changed_topic", fsm_state_changed_topic);
    getIntParam("status_msg_buffer_size", fsm_status_buffer_size);
    //Lidar topics
    getStringParam("lidar_topic", lidar_topic);
    //FSM topics
    getStringParam("local_position_topic", mavros_local_pos_topic);
    getStringParam("mavros_state_topic", mavros_state_changed_topic);
    //LandDetector
    getStringParam("land_detector_topic", land_detector_topic);
    //Obstacles
    getStringParam("obstacle_state_topic", obstacle_state_topic);
    //Obstacle avoidance
    getFloatParam("obstacle_clearance_side", obstacle_clearance_side);
    getFloatParam("obstacle_clearance_front", obstacle_clearance_front);
    getFloatParam("obstacle_clearance_back", obstacle_clearance_back);
    getFloatParam("obstacle_clearance_checkradius", obstacle_clearance_checkradius);
    
}

