#include "control_fsm/fsm_config.hpp"
#include <assert.h>

double FSMConfig::dest_reached_margin = 0.3;
double FSMConfig::blind_hover_alt = 1.0;
double FSMConfig::takeoff_altitude = 1.0;
double FSMConfig::altitude_reached_margin = 0.1;
double FSMConfig::setpoint_reached_margin = 0.3;
double FSMConfig::yaw_reached_margin = 0.02;
double FSMConfig::no_yaw_correct_dist = 0.2;
double FSMConfig::valid_data_timeout = 2.0; //2 seconds
std::string FSMConfig::fsm_error_topic = "control/fsm/on_error";
std::string FSMConfig::fsm_warn_topic = "control/fsm/on_warn";
std::string FSMConfig::fsm_info_topic = "control/fsm/on_info";
std::string FSMConfig::fsm_state_changed_topic = "control/fsm/state_changed";
std::string FSMConfig::mavros_local_pos_topic = "mavros/local_position/pose";
std::string FSMConfig::mavros_state_changed_topic = "mavros/state";
std::string FSMConfig::land_detector_topic = "/landdetector";
int FSMConfig::fsm_status_buffer_size = 10;
double FSMConfig::go_to_hold_dest_time = 0.5;
double FSMConfig::safe_hover_altitude = 2.0;
double FSMConfig::obstacle_too_close_dist = 2.0;
std::string FSMConfig::lidar_topic = "perception/obstacles/lidar";
bool FSMConfig::require_all_data_streams = true;
bool FSMConfig::require_obstacle_detection = true;

void FSMConfig::loadParams() {
    ros::NodeHandle n("~");
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

}
