#include "control_fsm/FSMConfig.hpp"
#include <assert.h>

double FSMConfig::DestReachedMargin = 0.3;
double FSMConfig::BlindHoverAlt = 1.0;
double FSMConfig::TakeoffAltitude = 1.0;
double FSMConfig::AltitudeReachedMargin = 0.1;
double FSMConfig::SetpointReachedMargin = 0.3;
double FSMConfig::YawReachedMargin = 0.02;
double FSMConfig::NoYawCorrectDist = 0.2;
std::string FSMConfig::PathPlannerPlanTopic = "control/planner/plan";
std::string FSMConfig::PathPlannerPosTopic = "control/planner/position";
std::string FSMConfig::PathPlannerTargetTopic = "control/planner/target";
std::string FSMConfig::PathPlannerObsTopic = "control/planner/obstacles";
std::string FSMConfig::FSMErrorTopic = "control/fsm/on_error";
std::string FSMConfig::FSMWarnTopic = "control/fsm/on_warn";
std::string FSMConfig::FSMInfoTopic = "control/fsm/on_info";
std::string FSMConfig::FSMStateChangedTopic = "control/fsm/state_changed";
std::string FSMConfig::MavrosLocalPosTopic = "mavros/local_position/pose";
std::string FSMConfig::MavrosStateChangedTopic = "mavros/state";
int FSMConfig::FSMStatusBufferSize = 10;
double FSMConfig::GoToHoldDestTime = 0.5;
double FSMConfig::SafeHoverAltitude = 2.0;
double FSMConfig::ObstacleTooCloseDist = 2.0;
std::string FSMConfig::LidarTopic = "perception/obstacles/lidar";
bool FSMConfig::RequireAllDataStreams = true;

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
	getDoubleParam("blind_hover_altitude", BlindHoverAlt);
	getDoubleParam("takeoff_altitude", TakeoffAltitude);
	getDoubleParam("altitude_reached_margin", AltitudeReachedMargin);
	getDoubleParam("yaw_reached_margin", YawReachedMargin);
	getDoubleParam("safe_hover_alt", SafeHoverAltitude);
	getDoubleParam("obstacle_too_close_dist", ObstacleTooCloseDist);
	getBoolParam("require_all_streams", RequireAllDataStreams);
	//GoTo params
	getDoubleParam("goto_hold_dest_time", GoToHoldDestTime);
	getDoubleParam("setp_reached_margin", SetpointReachedMargin);
	getDoubleParam("dest_reached_margin", DestReachedMargin);
	getDoubleParam("no_yaw_correct_dist", NoYawCorrectDist);
	//Planner params
	getStringParam("control_planner_plan", PathPlannerPlanTopic);
	getStringParam("control_planner_position", PathPlannerPosTopic);
	getStringParam("control_planner_target", PathPlannerTargetTopic);
	getStringParam("control_planner_obstacles", PathPlannerObsTopic);
	//FSM debug params
	getStringParam("fsm_error_topic", FSMErrorTopic);
	getStringParam("fsm_warn_topic", FSMWarnTopic);
	getStringParam("fsm_info_topic", FSMInfoTopic);
	getStringParam("fsm_state_changed_topic", FSMStateChangedTopic);
	getIntParam("status_msg_buffer_size", FSMStatusBufferSize);
	//Lidar topics
	getStringParam("lidar_topic", LidarTopic);
	//FSM topics
	getStringParam("local_position_topic", MavrosLocalPosTopic);
	getStringParam("mavros_state_topic", MavrosStateChangedTopic);

}