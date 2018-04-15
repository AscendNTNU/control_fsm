#include "control/tools/config.hpp"
#include <control/exceptions/ros_not_initialized_exception.hpp>
#include <ascend_msgs/ReloadConfig.h>
#include <control/tools/logger.hpp>

using control::Config;
using control::ROSNotInitializedException;


std::set<std::string> Config::missing_param_set_;
std::unique_ptr<Config> Config::shared_instance_p_ = nullptr;

//Global config params
double Config::dest_reached_margin = 0.3;
double Config::blind_hover_alt = 1.0;
double Config::takeoff_altitude = 1.0;
double Config::low_alt_takeoff_marg = 1.0;
double Config::land_xy_goto_alt = 1.0;
double Config::altitude_reached_margin = 0.1;
double Config::setpoint_reached_margin = 0.3;
double Config::yaw_reached_margin = 0.02;
double Config::gb_search_altitude = 2.5;
double Config::min_in_air_alt = 0.5;
double Config::no_yaw_correct_dist = 0.2;
double Config::valid_data_timeout = 2.0; //2 seconds
double Config::velocity_reached_margin = 0.2;
std::string Config::mavros_local_vel_topic = "mavros/local_position/velocity";
std::string Config::fsm_error_topic = "control/fsm/on_error";
std::string Config::fsm_warn_topic = "control/fsm/on_warn";
std::string Config::fsm_info_topic = "control/fsm/on_info";
std::string Config::fsm_state_changed_topic = "control/fsm/state_changed";
std::string Config::mavros_local_pos_topic = "mavros/local_position/pose";
std::string Config::mavros_state_changed_topic = "mavros/state";
std::string Config::land_detector_topic = "/landdetector";
std::string Config::land_detector_type = "landing_gear";
std::string Config::obstacle_state_topic = "/perception_obstacle_states";
std::string Config::debug_server_topic = "/control/fsm/debug_server";
std::string Config::action_server_topic = "/control/fsm/action_server";
int Config::fsm_status_buffer_size = 10;
double Config::go_to_hold_dest_time = 0.5;
double Config::safe_hover_altitude = 2.0;
double Config::obstacle_too_close_dist = 2.0;
std::string Config::lidar_topic = "perception/obstacles/lidar";
bool Config::require_all_data_streams = true;
bool Config::require_obstacle_detection = true;
std::string Config::global_frame_id = "map";
std::string Config::local_frame_id = "odom";
bool Config::use_global_transforms = true;

bool Config::restrict_arena_boundaries = true;
double Config::arena_lowest_x = 0.0;
double Config::arena_lowest_y = 0.0;
double Config::arena_highest_x = 20.0;
double Config::arena_highest_y = 20.0;

void Config::loadParams() {
    if(!ros::isInitialized()) {
        throw ROSNotInitializedException();
    }
    if(shared_instance_p_ == nullptr) {
        shared_instance_p_ = std::unique_ptr<Config>(new Config);
    } 
    ros::NodeHandle n("~");
    auto getDoubleParam = [&](const std::string& name, double& var, double min, double max) {
        double temp;
        if(!n.getParam(name, temp) || temp < min || temp > max) {
            std::string warn_msg = "Load param failed: ";
            warn_msg += name;
            warn_msg += ", using ";
            warn_msg += std::to_string(var);
            control::handleWarnMsg(warn_msg);
            missing_param_set_.insert(name);
        } else {
            var = temp;
            control::handleInfoMsg("Param " + name + " loaded: " + std::to_string(var));
        }
    };

    auto getStringParam = [&](const std::string& name, std::string& var) {
        if(!n.getParam(name, var)) {
            std::string warn_msg = "Load param failed: ";
            warn_msg += name;
            warn_msg += ", using ";
            warn_msg += var;
            control::handleWarnMsg(warn_msg);
            missing_param_set_.insert(name);
        } else {
            control::handleInfoMsg("Param " + name + " loaded: " + var);
        }
    };

    auto getIntParam = [&](const std::string& name, int& var, int min, int max) {
        int temp;
        if(!n.getParam(name, temp) || temp < min || temp > max) {
            std::string warn_msg = "Load param failed: ";
            warn_msg += name;
            warn_msg += ", using ";
            warn_msg += std::to_string(var);
            control::handleWarnMsg(warn_msg);
            missing_param_set_.insert(name);
        } else {
            var = temp;
            control::handleInfoMsg("Param " + name + " loaded: " + std::to_string(var));
        }
    };

    auto getBoolParam = [&](const std::string& name, bool& var) {
        if(!n.getParam(name, var)) {
            std::string warn_msg = "Load param failed: ";
            warn_msg += name;
            warn_msg += ", using ";
            warn_msg += std::to_string(var);
            control::handleWarnMsg(warn_msg);
            missing_param_set_.insert(name);
        } else {
            control::handleInfoMsg("Param " + name + " loaded: " + std::to_string(var));
        }
    };

    //Global params (used by multiple states)
    getDoubleParam("blind_hover_altitude", blind_hover_alt, 0.0, 5.0);
    getDoubleParam("takeoff_altitude", takeoff_altitude, 0.0, 5.0);
    getDoubleParam("low_alt_takeoff_marg", low_alt_takeoff_marg, 0.0, 1.0);
    getDoubleParam("altitude_reached_margin", altitude_reached_margin, 0.0, 1.0);
    getDoubleParam("yaw_reached_margin", yaw_reached_margin, 0.0, 0.2);
    getDoubleParam("safe_hover_alt", safe_hover_altitude, 0.0, 5.0);
    getDoubleParam("obstacle_too_close_dist", obstacle_too_close_dist, 0.0, 10.0);
    getDoubleParam("gb_search_altitude", gb_search_altitude, 0.0, 5.0);
    getDoubleParam("message_timeout", valid_data_timeout, 0.0, 60.0);
    getDoubleParam("min_in_air_alt", min_in_air_alt, 0.0, 5.0);
    getDoubleParam("velocity_reached_margin", velocity_reached_margin, 0.0, 1.0);
    getBoolParam("require_all_streams", require_all_data_streams);
    getBoolParam("require_obs_detection", require_obstacle_detection);
    //GoTo params
    getDoubleParam("goto_hold_dest_time", go_to_hold_dest_time, 0.0, 60.0);
    getDoubleParam("setp_reached_margin", setpoint_reached_margin, 0.0, 1.0);
    getDoubleParam("dest_reached_margin", dest_reached_margin, 0.0, 1.0);
    getDoubleParam("no_yaw_correct_dist", no_yaw_correct_dist, 0.0, 5.0);
    //FSM debug params
    getStringParam("fsm_error_topic", fsm_error_topic);
    getStringParam("fsm_warn_topic", fsm_warn_topic);
    getStringParam("fsm_info_topic", fsm_info_topic);
    getStringParam("fsm_state_changed_topic", fsm_state_changed_topic);
    getIntParam("status_msg_buffer_size", fsm_status_buffer_size, 1, 1000000);
    getStringParam("debug_server_topic", debug_server_topic);
    //Lidar topics
    getStringParam("lidar_topic", lidar_topic);
    //FSM topics
    getStringParam("local_position_topic", mavros_local_pos_topic);
    getStringParam("mavros_state_topic", mavros_state_changed_topic);
    getStringParam("local_velocity_topic", mavros_local_vel_topic);
    //LandDetector
    getStringParam("land_detector_topic", land_detector_topic);
    getStringParam("land_detector_type", land_detector_type);
    //Obstacles
    getStringParam("obstacle_state_topic", obstacle_state_topic);
    //Frame id
    getStringParam("global_frame_id", global_frame_id);
    getStringParam("local_frame_id", local_frame_id);
    getBoolParam("use_global_transforms", use_global_transforms);
    //Action server
    getStringParam("action_server_topic", action_server_topic);
    //Arena params
    getBoolParam("restrict_arena_boundaries", restrict_arena_boundaries);
    getDoubleParam("arena_lowest_x", arena_lowest_x, -100.0, 100.0);
    getDoubleParam("arena_lowest_y", arena_lowest_y, -100.0, 100.0);
    getDoubleParam("arena_highest_x", arena_highest_x, -100.0, 100.0);
    getDoubleParam("arena_highest_y", arena_highest_y, -100.0, 100.0);

}

using Request = ascend_msgs::ReloadConfig::Request;
using Response = ascend_msgs::ReloadConfig::Response;
bool reloadConfigCB(Request&, Response& resp) {
    control::Config::loadParams();
    control::handleInfoMsg("Config reloaded by service!");

    //Missing param set should be empty!
    for(auto& s : control::Config::getMissingParamSet()) {
        resp.missing_params.emplace_back(s);
    }
    return true;
}

control::Config::Config() {
    if(!ros::isInitialized()) {
        throw control::ROSNotInitializedException();
    }
    reload_config_service = nh_.advertiseService("/control_fsm_reload_config", reloadConfigCB);
}
