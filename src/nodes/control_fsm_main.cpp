#include <ros/ros.h>

#include "control_fsm/ControlFSM.hpp"
#include "control_fsm/ActionServer.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <ascend_msgs/ControlFSMEvent.h>
#include <std_msgs/String.h>


bool firstPositionRecieved = false;
bool isArmed = false;
bool isOffboard = false;

constexpr float SETPOINT_PUB_RATE = 30.0f; //In Hz

std::string fsmOnStateChangedPubTopic = "control/fsm/state_changed";
std::string fsmErrorPubTopic = "control/fsm/on_error";
std::string fsmWarnPubTopic = "control/fsm/on_warn";
std::string fsmInfoPubTopic = "control/fsm/on_info";
int statusMsgBufferSize = 10;

ControlFSM fsm;

void localPosCB(const geometry_msgs::PoseStamped& input);
void mavrosStateChangedCB(const mavros_msgs::State& state);

EventData generateDebugEvent(ascend_msgs::ControlFSMEvent::Request&);
bool handleDebugEvent(ascend_msgs::ControlFSMEvent::Request&, ascend_msgs::ControlFSMEvent::Response&);

void loadParams(ros::NodeHandle& n);


int main(int argc, char** argv) {

	ros::init(argc, argv, "control_fsm_main");
	ros::NodeHandle n;
	ros::NodeHandle np("~");

	//Load ros params
	loadParams(np);

	//Subscribe to neccesary topics
	ros::Subscriber localPosSub = n.subscribe("mavros/local_position/pose", 1, localPosCB);
	ros::Subscriber mavrosStateChangedSub = n.subscribe("mavros/state", 1, mavrosStateChangedCB);
	
	//Set up neccesary publishers
	ros::Publisher setpointPub = n.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);
	ros::Publisher fsmOnStateChangedPub = n.advertise<std_msgs::String>(fsmOnStateChangedPubTopic, statusMsgBufferSize);
	ros::Publisher fsmOnErrorPub = n.advertise<std_msgs::String>(fsmErrorPubTopic, statusMsgBufferSize);
	ros::Publisher fsmOnInfoPub = n.advertise<std_msgs::String>(fsmInfoPubTopic, statusMsgBufferSize);
	ros::Publisher fsmOnWarnPub = n.advertise<std_msgs::String>(fsmWarnPubTopic, statusMsgBufferSize);

	//Set up services
	ros::ServiceServer debugServiceServer = n.advertiseService("control_fsm_debug", handleDebugEvent);

	//Spin once to get first messages
	ros::spinOnce();

	//Set FSM callbacks
	fsm.setOnStateChangedCB([&](){
		std_msgs::String msg;
		msg.data = fsm.getState()->getStateName();
		fsmOnStateChangedPub.publish(msg);
	});

	fsm.setOnFSMErrorCB([&](const std::string& errMsg) {
		std_msgs::String msg;
		msg.data = errMsg;
		fsmOnErrorPub.publish(msg);
	});

	fsm.setOnFSMWarnCB([&](const std::string& warnMsg) {
		std_msgs::String msg;
		msg.data = warnMsg;
		fsmOnWarnPub.publish(msg);
	});

	fsm.setOnFSMInfoCB([&](const std::string& infoMsg) {
		std_msgs::String msg;
		msg.data = infoMsg;
		fsmOnInfoPub.publish(msg);
	});

	//Wait for all systems to initalize and position to become valid
	ROS_INFO("Waiting for first position msg!");
	while(ros::ok() && !firstPositionRecieved) {
		ros::spinOnce();
		ros::Duration(0.5).sleep();
	}
	ROS_INFO("First position message recieved!");

	//Set up actionserver
	ActionServer cmdServer(&fsm);

	//Used to maintain a fixed loop rate
	ros::Rate loopRate(SETPOINT_PUB_RATE);

	while(ros::ok()) {
		//TODO Take get input from planning or other 
		//TODO Implement actionlib

		ros::spinOnce(); //Handle all incoming messages - generates fsm events
		fsm.loopCurrentState(); //Run current FSM state loop

		//Publish setpoints at gived rate
		const mavros_msgs::PositionTarget* pSetpoint = fsm.getSetpoint();
		setpointPub.publish(*pSetpoint);

		//Sleep for remaining time
 		loopRate.sleep();
	}

	return 0;
}

void localPosCB(const geometry_msgs::PoseStamped& input) {
	fsm.setPosition(input);
	if(!firstPositionRecieved) {
		firstPositionRecieved = true;
	}
}

void mavrosStateChangedCB(const mavros_msgs::State& state) {
	bool offboardTrue = (state.mode == "OFFBOARD");
	//Only act if relevant states has changed
	if(offboardTrue != isOffboard || state.armed != isArmed) {
		//Check if old state was autonomous
		//=> now in manual mode
		if(isOffboard && isArmed) {
			EventData manualEvent;
			manualEvent.eventType = EventType::MANUAL;
			fsm.handleEvent(manualEvent);
		}
		//Set current state
		isOffboard = offboardTrue;
		isArmed = state.armed;

		//If it is armed and in offboard - notify AUTONOMOUS mode
		if(isArmed && isOffboard) {
			EventData autonomousEvent;
			autonomousEvent.eventType = EventType::AUTONOMOUS;
			fsm.handleEvent(autonomousEvent);
		}
	}
}

bool handleDebugEvent(ascend_msgs::ControlFSMEvent::Request& req, ascend_msgs::ControlFSMEvent::Response& resp) {
	EventData event = generateDebugEvent(req);
	if(event.eventType == EventType::REQUEST && !event.isValidRequest()) {
		resp.accepted = false;
		resp.errorMsg = "Not valid request event";
		return true;
	} 
	if(event.eventType == EventType::COMMAND && !event.isValidCMD()) {
		resp.accepted = false;
		resp.errorMsg = "Not valid cmd event";
		return true;
	}
	if(event.eventType == EventType::NONE) {
		resp.accepted = false;
		resp.errorMsg = "No valid event";
		return true;
	}

	if(event.isValidCMD()) {
		event.setOnCompleteCallback([](){
			ROS_INFO("Manual CMD finished");
		});
		event.setOnErrorCallback([](std::string errMsg) {
			ROS_WARN("Manual CMD error: %s", errMsg.c_str());
		});
	}
	fsm.handleEvent(event);
	resp.accepted = true;
	resp.stateName = fsm.getState()->getStateName();
	return true;

}

EventData generateDebugEvent(ascend_msgs::ControlFSMEvent::Request&req) {
	EventData event;
	//Lambda expression returning correct eventtype
	event.eventType = ([&]() -> EventType{
		switch(req.eventType) {
			case req.REQUEST: return EventType::REQUEST;
			case req.COMMAND: return EventType::COMMAND;
			case req.AUTONOMOUS: return EventType::AUTONOMOUS;
			case req.MANUAL: return EventType::MANUAL;
			case req.GROUNDDETECTED: return EventType::GROUNDDETECTED;
			default: return EventType::NONE;
		}
	})();

	if(event.eventType == EventType::REQUEST) {
		//Lambda expression returning correct requesttype
		event.request = ([&]() -> RequestType {
			switch(req.requestType) {
				case req.ABORT: return RequestType::ABORT;
				case req.BEGIN: return RequestType::BEGIN;
				case req.END: return RequestType::END;
				case req.PREFLIGHT: return RequestType::PREFLIGHT;
				case req.IDLE: return RequestType::IDLE;
				case req.SHUTDOWN: return RequestType::SHUTDOWN;
				case req.TAKEOFF: return RequestType::TAKEOFF;
				case req.BLINDHOVER: return RequestType::BLINDHOVER;
				case req.POSHOLD: return RequestType::POSHOLD;
				case req.GOTO: return RequestType::GOTO;
				case req.LAND: return RequestType::LAND;
				case req.BLINDLAND: return RequestType::BLINDLAND;
				//case req.TRACKGB: return RequestType::TRACKGB;
				//case req.INTERGB: return RequestType::INTERGB;
				case req.ESTIMATORADJ: return RequestType::ESTIMATORADJ;
				default: return RequestType::NONE;
			}
		})();
	} else if(event.eventType == EventType::COMMAND) {
		//Lambda expression returning correct commandEvent
		event = ([&]() -> EventData{
			switch(req.commandType) {
				case req.LANDXY: return LandXYCMDEvent(req.x, req.y, req.yaw);
				case req.GOTOXYZ: return GoToXYZCMDEvent(req.x, req.y, req.z, req.yaw);
				//case req.LANDGB: return LandGBCMDEvent();
				default:
					EventData e;
					e.eventType = EventType::COMMAND;
					e.commandType = CommandType::NONE;
					return e;
			}
		})();
	}
	return event;
}

void loadParams(ros::NodeHandle& n) {
	if(!n.getParam("fsm_error_topic", fsmErrorPubTopic)) {
		ROS_WARN("Param fsm_error_topic not found");
	}
	ROS_INFO("FSM error topic: %s", fsmErrorPubTopic.c_str());

	if(!n.getParam("fsm_warn_topic", fsmWarnPubTopic)) {
		ROS_WARN("Param fsm_warn_topic not found");
	}
	ROS_INFO("FSM warn topic: %s", fsmWarnPubTopic.c_str());

	if(!n.getParam("fsm_info_topic", fsmInfoPubTopic)) {
		ROS_WARN("Param fsm_info_topic not found");
	}
	ROS_INFO("FSM info topic: %s", fsmInfoPubTopic.c_str());

	if(!n.getParam("fsm_state_changed_topic", fsmOnStateChangedPubTopic)) {
		ROS_WARN("Param fsm_state_changed_topic not found");
	}
	ROS_INFO("FSM state changed topic: %s", fsmOnStateChangedPubTopic.c_str());
	if(!n.getParam("status_msg_buffer_size", statusMsgBufferSize)) {
		ROS_WARN("Param status_msg_buffer_size not found");
	}
	ROS_INFO("Status msg buffer size: %i", statusMsgBufferSize);
}