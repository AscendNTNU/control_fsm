#include "control_fsm/ControlFSM.hpp"
#include <iostream>
#include <ros/ros.h>

#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/Int32.h>

#include "control_fsm/ControlFSM.hpp"
#include "control_fsm/EventData.hpp"

mavros_msgs::State current_state;

bool first_position_recieved = false;
bool is_armed = false;
bool is_offboard = false;

ControlFSM fsm;

void local_pos_callback(const geometry_msgs::PoseStamped& input);
void state_callback(const mavros_msgs::State::ConstPtr& msg);
void user_input_callback(const std_msgs::Int32& val);

int main(int argc, char** argv) {

	ros::init(argc, argv, "control_fsm_gazebo_test");
	
	//Subscribe to neccesary topics
	ros::NodeHandle n;
	ros::Subscriber local_pos_sub = n.subscribe("mavros/local_position/pose", 10, local_pos_callback);
	ros::Subscriber user_input_sub = n.subscribe("control_fsm_gazebo_input/value", 10, user_input_callback);

	ros::Publisher setpoint_pub = n.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, state_callback);

	ros::spinOnce();

	ros::Rate loop_rate(20);

	mavros_msgs::PositionTarget tempSetpoint;
	tempSetpoint.position.x = 0;
	tempSetpoint.position.y = 0;
	tempSetpoint.position.z = 0;

	//Wait for offboard mode and arming
	while ( !is_offboard || !is_armed) {
		setpoint_pub.publish(tempSetpoint);
		ros::spinOnce();
		loop_rate.sleep();
	}

	if(is_offboard && is_armed) {
		EventData armedEvent;
		armedEvent.eventType = EventType::ARMED;
		fsm.handleEvent(armedEvent);
	} else {
		return -1;
	}

	ros::Time setpoint_last_sent = ros::Time::now();
	while(ros::ok()) {
		ros::spinOnce();
		fsm.loopCurrentState();
		if(ros::Time::now() - setpoint_last_sent >= ros::Duration(1/30.0)) {
			const mavros_msgs::PositionTarget* pSetpoint = fsm.getSetpoint();
			setpoint_pub.publish(*pSetpoint);
		}
	}

	return 0;
}

void local_pos_callback(const geometry_msgs::PoseStamped& input) {
	//Faking ground detected event
	if(fsm.getPositionZ() >= 0.2 && input.pose.position.z < 0.2) {
		EventData landEvent;
		landEvent.eventType = EventType::GROUNDDETECTED;
		fsm.handleEvent(landEvent);
	}
	fsm.setPosition(input);
}

void state_callback(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
	if(!is_armed && msg->armed) {
		is_armed = msg->armed;
		ROS_INFO("ARMED");
	} else if(is_armed && !msg->armed) {
		is_armed = msg->armed;
		ROS_INFO("DISARMED");
	}
	bool ofb = (msg->mode == "OFFBOARD");
	if(!is_offboard && ofb) {
		is_offboard = (msg->mode == "OFFBOARD");
		ROS_INFO("OFFBOARD");
	} else if(is_offboard && !ofb) {
		is_offboard = false;
		ROS_INFO("LOST OFFBOARD");
	}

}

//Generate events based on user input
void user_input_callback(const std_msgs::Int32& val) {
	EventData event;
		switch(val.data) {
			case 1:
				event.eventType = EventType::REQUEST;
				event.request = RequestType::NONE;
				break;
			case 2:
				event.eventType = EventType::REQUEST;
				event.request = RequestType::ABORT;
				break;
			case 3:
				event.eventType = EventType::REQUEST;
				event.request = RequestType::BEGIN;
				break;
			case 4:
				event.eventType = EventType::REQUEST;
				event.request = RequestType::END;
				break;
			case 5:
				event.eventType = EventType::REQUEST;
				event.request = RequestType::PREFLIGHT;
				break;
			case 6:
				event.eventType = EventType::REQUEST;
				event.request = RequestType::IDLE;
				break;
			case 7:
				event.eventType = EventType::REQUEST;
				event.request = RequestType::SHUTDOWN;
				break;
			case 8:
				event.eventType = EventType::REQUEST;
				event.request = RequestType::TAKEOFF;
				break;
			case 9:
				event.eventType = EventType::REQUEST;
				event.request = RequestType::BLINDHOVER;
				break;
			case 10:
				event.eventType = EventType::REQUEST;
				event.request = RequestType::POSHOLD;
				break;
			case 11:
				event.eventType = EventType::REQUEST;
				event.request = RequestType::GOTO;
				break;
			case 12:
				event.eventType = EventType::REQUEST;
				event.request = RequestType::LAND;
				break;
			case 13:
				event.eventType = EventType::REQUEST;
				event.request = RequestType::BLINDLAND;
				break;
			case 14:
				event.eventType = EventType::REQUEST;
				event.request = RequestType::TRACKGB;
				break;
			case 15:
				event.eventType = EventType::REQUEST;
				event.request = RequestType::INTERGB;
				break;
			case 16:
				event.eventType = EventType::REQUEST;
				event.request = RequestType::ESTIMATORADJ;
				break;
			case 17:
				event.eventType = EventType::COMMAND;
				event.commandType = CommandType::NONE;
				event.setOnCompleteCallback([](){
					std::cout << "Command complete!" << std::endl;
				});
				event.setOnErrorCallback([&](std::string s) {
					std::cout << "Callback: Command error!" << std::endl;
				});
				break;
			case 18:
				event.eventType = EventType::COMMAND;
				event.commandType = CommandType::LANDXY;
				event.setOnCompleteCallback([](){
					std::cout << "Command complete!" << std::endl;
				});
				event.setOnErrorCallback([](std::string s) {
					std::cout << "Callback: Command error!" << std::endl;
				});
				event.positionGoal = PositionGoalXYZ(10, 10, 2, 0);
				break;
			case 19:
				event.eventType = EventType::COMMAND;
				event.commandType = CommandType::GOTOXYZ;
				event.setOnCompleteCallback([](){
					std::cout << "Command complete!" << std::endl;
				});
				event.setOnErrorCallback([](std::string s) {
					std::cout << "Callback: Command error!" << std::endl;
				});
				event.positionGoal = PositionGoalXYZ(10, 10, 2, 0);
				break;
			case 20:
				event.eventType = EventType::COMMAND;
				event.commandType = CommandType::LANDGB;
				event.setOnCompleteCallback([](){
					std::cout << "Callback: Command complete!" << std::endl;
				});
				event.setOnErrorCallback([](std::string s) {
					std::cout << "Callback: Command error!" << std::endl;
				});
				event.positionGoal = PositionGoalXYZ(10, 10, 2, 0);
				break;
			case 21:
				event.eventType = EventType::ARMED;
				break;
			case 22:
				event.eventType = EventType::DISARMED;
				break;
			case 23:
				event.eventType = EventType::POSREGAINED;
				break;
			case 24:
				event.eventType = EventType::POSLOST;
				break;
			case 25:
				event.eventType = EventType::GROUNDDETECTED;
				break;
			default:
				std::cout << "Not an option" << std::endl;
				break;
		}
		fsm.handleEvent(event);
}