#include <ros/ros.h>

#include "control_fsm/ControlFSM.hpp"
#include "control_fsm/ActionServer.hpp"
#include "control_fsm/FSMConfig.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <ascend_msgs/ControlFSMEvent.h>
#include <ascend_msgs/PointArray.h>
#include <std_msgs/String.h>
#include <cmath>


bool firstPositionRecieved = false;
bool isArmed = false;
bool isOffboard = false;

constexpr float SETPOINT_PUB_RATE = 30.0f; //In Hz

//Information topics
std::string fsmOnStateChangedPubTopic = "control/fsm/state_changed";
std::string fsmErrorPubTopic = "control/fsm/on_error";
std::string fsmWarnPubTopic = "control/fsm/on_warn";
std::string fsmInfoPubTopic = "control/fsm/on_info";
int statusMsgBufferSize = 10;

//Datatopics
std::string lidarTopic = "perception/obstacles/lidar"; //Topic is not set
std::string localPosTopic = "mavros/local_position/pose";
std::string mavrosStateTopic = "mavros/state";

//Global variables
double obstacleTooCloseDist = 2.0;
double safeHoverAlt = 2.5;

//Statemachine
ControlFSM fsm;

//Callback definitions
void localPosCB(const geometry_msgs::PoseStamped& input);
void mavrosStateChangedCB(const mavros_msgs::State& state);
void lidarCB(const ascend_msgs::PointArray& msg);

//Debug service functions
EventData generateDebugEvent(ascend_msgs::ControlFSMEvent::Request&);
bool handleDebugEvent(ascend_msgs::ControlFSMEvent::Request&, ascend_msgs::ControlFSMEvent::Response&);

//Loads ros parameters 
void loadParams(ros::NodeHandle& n);


int main(int argc, char** argv) {
	//Init ros and nodehandles
	ros::init(argc, argv, "control_fsm_main");
	ros::NodeHandle n;
	ros::NodeHandle np("~");

	//Load ros params
	FSMConfig::loadParams();

	//Subscribe to neccesary topics
	ros::Subscriber localPosSub = n.subscribe(localPosTopic, 1, localPosCB);
	ros::Subscriber mavrosStateChangedSub = n.subscribe(mavrosStateTopic, 1, mavrosStateChangedCB);
	ros::Subscriber lidarSub = n.subscribe(FSMConfig::LidarTopic, 1, lidarCB);

	//Set up neccesary publishers
	ros::Publisher setpointPub = n.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);
	ros::Publisher fsmOnStateChangedPub = n.advertise<std_msgs::String>(FSMConfig::FSMStateChangedTopic, statusMsgBufferSize);
	ros::Publisher fsmOnErrorPub = n.advertise<std_msgs::String>(FSMConfig::FSMErrorTopic, statusMsgBufferSize);
	ros::Publisher fsmOnInfoPub = n.advertise<std_msgs::String>(FSMConfig::FSMInfoTopic, statusMsgBufferSize);
	ros::Publisher fsmOnWarnPub = n.advertise<std_msgs::String>(FSMConfig::FSMWarnTopic, statusMsgBufferSize);

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
		ros::Duration(0.5).sleep();
		ros::spinOnce();
	}
	ROS_INFO("First position message recieved!");

	//Actionserver is started when the system is ready
	ActionServer cmdServer(&fsm);

	//Used to maintain a fixed loop rate
	ros::Rate loopRate(SETPOINT_PUB_RATE);
	//Main loop
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
	bool offboardTrue = (state.mode == std::string("OFFBOARD"));
	bool armedTrue = (bool)state.armed;
	//Only act if relevant states has changed
	if(offboardTrue != isOffboard || armedTrue != isArmed) {
		//Check if old state was autonomous
		//=> now in manual mode
		if(isOffboard && isArmed) {
			EventData manualEvent;
			manualEvent.eventType = EventType::MANUAL;
			ROS_INFO("Manual sent!");
			fsm.handleEvent(manualEvent);
		}
		//Set current state
		isOffboard = offboardTrue;
		isArmed = state.armed;

		//If it is armed and in offboard - notify AUTONOMOUS mode
		if(isArmed && isOffboard) {
			EventData autonomousEvent;
			autonomousEvent.eventType = EventType::AUTONOMOUS;
			ROS_INFO("Autonomous event sent");
			fsm.handleEvent(autonomousEvent);
		}
	}
}

bool handleDebugEvent(ascend_msgs::ControlFSMEvent::Request& req, ascend_msgs::ControlFSMEvent::Response& resp) {
	EventData event = generateDebugEvent(req);
	//If request event is not valid
	if(event.eventType == EventType::REQUEST && !event.isValidRequest()) {
		resp.accepted = false;
		resp.errorMsg = "Not valid request event";
		return true;
	} 
	//If command event is not valid
	if(event.eventType == EventType::COMMAND && !event.isValidCMD()) {
		resp.accepted = false;
		resp.errorMsg = "Not valid cmd event";
		return true;
	}
	//If there is no valid event
	if(event.eventType == EventType::NONE) {
		resp.accepted = false;
		resp.errorMsg = "No valid event";
		return true;
	}

	if(event.isValidCMD()) {
		event.setOnCompleteCallback([](){
			ROS_INFO("[Control FSM Debug] Manual CMD finished");
		});
		event.setOnFeedbackCallback([](std::string msg){
			ROS_INFO("[Control FSM Debug] Manual CMD feedback: %s", msg.c_str());
		});
		event.setOnErrorCallback([](std::string errMsg) {
			ROS_WARN("[Control FSM Debug] Manual CMD error: %s", errMsg.c_str());
		});
	}
	fsm.handleEvent(event);
	resp.accepted = true;
	resp.stateName = fsm.getState()->getStateName();
	return true;

}

EventData generateDebugEvent(ascend_msgs::ControlFSMEvent::Request&req) {
	EventData event;
	ROS_INFO("X: %f, Y: %f, Z: %f", req.x, req.y, req.z);
	//Lambda expression returning correct eventtype
	event.eventType = ([&]() -> EventType{
		typedef ascend_msgs::ControlFSMEvent::Request REQ;
		switch(req.eventType) {
			case REQ::REQUEST: return EventType::REQUEST;
			case REQ::COMMAND: return EventType::COMMAND;
			case REQ::AUTONOMOUS: return EventType::AUTONOMOUS;
			case REQ::MANUAL: return EventType::MANUAL;
			case REQ::GROUNDDETECTED: return EventType::GROUNDDETECTED;
			default: return EventType::NONE;
		}
	})();

	if(event.eventType == EventType::REQUEST) {
		//Lambda expression returning correct requesttype
		event.request = ([&]() -> RequestType {
			typedef ascend_msgs::ControlFSMEvent::Request REQ;
			switch(req.requestType) {
				case REQ::ABORT: return RequestType::ABORT;
				case REQ::BEGIN: return RequestType::BEGIN;
				case REQ::END: return RequestType::END;
				case REQ::PREFLIGHT: return RequestType::PREFLIGHT;
				case REQ::IDLE: return RequestType::IDLE;
				case REQ::SHUTDOWN: return RequestType::SHUTDOWN;
				case REQ::TAKEOFF: return RequestType::TAKEOFF;
				case REQ::BLINDHOVER: return RequestType::BLINDHOVER;
				case REQ::POSHOLD: return RequestType::POSHOLD;
				case REQ::GOTO: return RequestType::GOTO;
				case REQ::LAND: return RequestType::LAND;
				case REQ::BLINDLAND: return RequestType::BLINDLAND;
				//case REQ::TRACKGB: return RequestType::TRACKGB;
				//case REQ::INTERGB: return RequestType::INTERGB;
				case REQ::ESTIMATORADJ: return RequestType::ESTIMATORADJ;
				case REQ::MANUALFLIGHT: return RequestType::MANUALFLIGHT;
				default: return RequestType::NONE;
			}
		})();
		if(event.request == RequestType::GOTO) {
			event.positionGoal = PositionGoalXYZ(req.x, req.y, req.z);
		}
	} else if(event.eventType == EventType::COMMAND) {
		//Lambda expression returning correct commandEvent
		event = ([&]() -> EventData{
			typedef ascend_msgs::ControlFSMEvent::Request REQ;
			switch(req.commandType) {
				case REQ::LANDXY: return LandXYCMDEvent(req.x, req.y);
				case REQ::GOTOXYZ: return GoToXYZCMDEvent(req.x, req.y, req.z);
				//case REQ::LANDGB: return LandGBCMDEvent();
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

void lidarCB(const ascend_msgs::PointArray& msg) {
	static bool isTooClose = false;
	auto points = msg.points;
	const geometry_msgs::PoseStamped* pPose = fsm.getPositionXYZ();
	if(pPose == nullptr) {
		//No valid XY position available, no way to determine distance to GB
		return;
	}
	//No need to check obstacles if they're too close
	if(pPose->pose.position.z >= FSMConfig::SafeHoverAltitude) {
		return;
	}

	double droneX = pPose->pose.position.x;
	double droneY = pPose->pose.position.y;
	for(int i = 0; i < points.size(); ++i) {
		double distSquared = std::pow(droneX - points[i].x, 2) + std::pow(droneY - points[i].y, 2);
		if(distSquared < std::pow(FSMConfig::ObstacleTooCloseDist, 2)) {
			if(!isTooClose) {
				EventData event;
				event.eventType = EventType::OBSTACLECLOSING;
				fsm.handleEvent(event);
				isTooClose = true;
			}
			//If one is close enough, no need to check the rest
			return;
		}
	}
	isTooClose = false;
}
