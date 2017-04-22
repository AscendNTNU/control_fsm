#include <ros/ros.h>

#include "control_fsm/ControlFSM.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <ascend_msgs/ControlFSMEvent.h>


bool first_position_recieved = false;
bool is_armed = false;
bool is_offboard = false;

constexpr double SETPOINT_PUB_RATE = 30.0f; //In Hz


ControlFSM fsm;

void local_pos_callback(const geometry_msgs::PoseStamped& input);
void state_changed_callback(const mavros_msgs::State& state);

EventData generateDebugEvent(ascend_msgs::ControlFSMEvent::Request&);
bool handleDebugEvent(ascend_msgs::ControlFSMEvent::Request&, ascend_msgs::ControlFSMEvent::Response&);


int main(int argc, char** argv) {

	ros::init(argc, argv, "control_fsm_main");
	
	//Subscribe to neccesary topics
	ros::NodeHandle n;
	ros::Subscriber local_pos_sub = n.subscribe("mavros/mocap/pose", 10, local_pos_callback);
	
	ros::Publisher setpoint_pub = n.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

	ros::ServiceServer debugServiceServer = n.advertiseService("control_fsm_debug", handleDebugEvent);

	ros::spinOnce();

	ros::Rate loop_rate(30);

	//Wait for all systems to initalize
	while(ros::ok() && !first_position_recieved) {
		ros::spinOnce();
	}

	ros::Time setpointLastPub = ros::Time::now();
	while(ros::ok()) {
		//TODO Take get input from planning or other 
		//TODO Implement actionlib

		ros::spinOnce(); //Handle all incoming messages
		fsm.loopCurrentState(); //Run current FSM state loop
		if(ros::Time::now() - setpointLastPub >= ros::Duration(1.0 / SETPOINT_PUB_RATE)) {
			const mavros_msgs::PositionTarget* pSetpoint = fsm.getSetpoint();
			setpoint_pub.publish(*pSetpoint);
			setpointLastPub = ros::Time::now();
		}
	}

	return 0;
}

void local_pos_callback(const geometry_msgs::PoseStamped& input) {
	fsm.setPosition(input);
}

void state_changed_callback(const mavros_msgs::State& state) {
	bool offboardTrue = (state.mode == "OFFBOARD");
	//Only act if relevant states has changed
	if(offboardTrue != is_offboard || state.armed != is_armed) {
		if(is_offboard && is_armed) {
			EventData manualEvent;
			manualEvent.eventType = EventType::MANUAL;
			fsm.handleEvent(manualEvent);
		}

		is_offboard = offboardTrue;
		is_armed = state.armed;
		if(is_armed && is_offboard) {
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