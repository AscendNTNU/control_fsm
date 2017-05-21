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
bool preflightFinished = false;

//How often is setpoints published to flightcontroller?
constexpr float SETPOINT_PUB_RATE = 30.0f; //In Hz

//Mavros topics -
constexpr char localPosTopic[] = "mavros/local_position/pose";
constexpr char mavrosStateTopic[] = "mavros/state";
constexpr char mavrosSetpointTopic[] = "mavros/setpoint_raw/local";
//Statemachine
ControlFSM fsm;

//Callback definitions
void localPosCB(const geometry_msgs::PoseStamped& input);
void mavrosStateChangedCB(const mavros_msgs::State& state);
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

	///Init fsm
	fsm.init();

	//Subscribe to neccesary topics
	ros::Subscriber localPosSub = n.subscribe(localPosTopic, 1, localPosCB);
	ros::Subscriber mavrosStateChangedSub = n.subscribe(mavrosStateTopic, 1, mavrosStateChangedCB);

	//Set up neccesary publishers
	ros::Publisher setpointPub = n.advertise<mavros_msgs::PositionTarget>(mavrosSetpointTopic, 1);
	ros::Publisher fsmOnStateChangedPub = n.advertise<std_msgs::String>(FSMConfig::FSMStateChangedTopic, FSMConfig::FSMStatusBufferSize);
	ros::Publisher fsmOnErrorPub = n.advertise<std_msgs::String>(FSMConfig::FSMErrorTopic, FSMConfig::FSMStatusBufferSize);
	ros::Publisher fsmOnInfoPub = n.advertise<std_msgs::String>(FSMConfig::FSMInfoTopic, FSMConfig::FSMStatusBufferSize);
	ros::Publisher fsmOnWarnPub = n.advertise<std_msgs::String>(FSMConfig::FSMWarnTopic, FSMConfig::FSMStatusBufferSize);

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
	fsm.handleFSMInfo("Waiting for necessary data streams!");
	while(ros::ok()) {
		ros::Duration(0.5).sleep();
		ros::spinOnce();
		//Make sure all critical datastreams are up and running before continuing
		if(!firstPositionRecieved) continue;
		if(mavrosStateChangedSub.getNumPublishers() <= 0) continue;
		if(!fsm.isReady()) continue; //Checks if all states are ready
		break;
	}
	fsm.handleFSMInfo("Necessary data streams are ready!");

	//Actionserver is started when the system is ready
	ActionServer cmdServer(&fsm);

    preflightFinished = true;

    //Preflight is finished and system is ready for use!
    /**************************************************/
    fsm.handleFSMInfo("FSM is ready!");
	fsm.startPreflight(); //Transition to preflight!
	//Used to maintain a fixed loop rate
	ros::Rate loopRate(SETPOINT_PUB_RATE);
	//Main loop
	while(ros::ok()) {
        //Get latest messages
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

		//If it is armed and in offboard and all preflight checks has completed - notify AUTONOMOUS mode
		if(isArmed && isOffboard && preflightFinished) {
			EventData autonomousEvent;
			autonomousEvent.eventType = EventType::AUTONOMOUS;
			ROS_INFO("Autonomous event sent");
			fsm.handleEvent(autonomousEvent);
		}
	}
}

bool handleDebugEvent(ascend_msgs::ControlFSMEvent::Request& req, ascend_msgs::ControlFSMEvent::Response& resp) {
    if(!preflightFinished) {
        fsm.handleFSMWarn("Preflight not complete, however FSM do respond to debug requests! Be careful!!");
    }
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
			ROS_INFO("[Control FSM Debug] Manual CMD finished");
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
	//Lambda expression returning correct eventtype
	event.eventType = ([&]() -> EventType{
		using REQ = ascend_msgs::ControlFSMEvent::Request;
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
			using REQ = ascend_msgs::ControlFSMEvent::Request;
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
			using REQ = ascend_msgs::ControlFSMEvent::Request;
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

