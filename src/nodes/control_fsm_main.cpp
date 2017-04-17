#include <ros/ros.h>

#include "control_fsm/ControlFSM.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>


bool first_position_recieved = false;
bool is_armed = false;
bool is_offboard = false;

constexpr float SETPOINT_PUB_RATE = 1.0 / 30.0f; //In seconds


ControlFSM fsm;

void local_pos_callback(const geometry_msgs::PoseStamped& input);
void state_changed_callback(const mavros_msgs::State& state);

int main(int argc, char** argv) {

	ros::init(argc, argv, "control_fsm_main");
	
	//Subscribe to neccesary topics
	ros::NodeHandle n;
	ros::Subscriber local_pos_sub = n.subscribe("mavros/mocap/pose", 10, local_pos_callback);
	
	//Set up neccesary publishers
	ros::Publisher setpoint_pub = n.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

	//Spin once to get first messages
	ros::spinOnce();

	//Wait for all systems to initalize and position to become valid
	while(ros::ok() && !first_position_recieved) {
		ros::spinOnce();
	}

	ros::Time setpointLastPub = ros::Time::now();
	while(ros::ok()) {
		//TODO Take get input from planning or other 
		//TODO Implement actionlib

		ros::spinOnce(); //Handle all incoming messages - generates fsm events
		fsm.loopCurrentState(); //Run current FSM state loop

		//Publish setpoints at gived rate
		if(ros::Time::now() - setpointLastPub >= ros::Duration(SETPOINT_PUB_RATE)) {
			constq mavros_msgs::PositionTarget* pSetpoint = fsm.getSetpoint();
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
		//Check if old state was autonomous
		//=> now in manual mode
		if(is_offboard && is_armed) {
			EventData manualEvent;
			manualEvent.eventType = EventType::MANUAL;
			fsm.handleEvent(manualEvent);
		}
		//Set current state
		is_offboard = offboardTrue;
		is_armed = state.armed;

		//If it is armed and in offboard - notify AUTONOMOUS mode
		if(is_armed && is_offboard) {
			EventData autonomousEvent;
			autonomousEvent.eventType = EventType::AUTONOMOUS;
			fsm.handleEvent(autonomousEvent);
		}
	}
}