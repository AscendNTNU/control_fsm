#include <ros/ros.h>

#include "control_fsm/ControlFSM.hpp"
#include "control_fsm/ActionServer.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>


bool firstPositionRecieved = false;
bool isArmed = false;
bool isOffboard = false;

constexpr float SETPOINT_PUB_RATE = 30.0f; //In Hz


ControlFSM fsm;

void localPosCB(const geometry_msgs::PoseStamped& input);
void mavrosStateChangedCB(const mavros_msgs::State& state);

int main(int argc, char** argv) {

	ros::init(argc, argv, "control_fsm_main");
	ros::NodeHandle n;
	//Subscribe to neccesary topics
	ros::Subscriber localPosSub = n.subscribe("mavros/local_position/pose", 1, localPosCB);
	ros::Subscriber mavrosStateChangedSub = n.subscribe("mavros/state", 1, mavrosStateChangedCB);
	
	//Set up neccesary publishers
	ros::Publisher setpointPub = n.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);

	//Spin once to get first messages
	ros::spinOnce();

	//Wait for all systems to initalize and position to become valid
	ROS_INFO("Waiting for first position msg!");
	while(ros::ok() && !firstPositionRecieved) {
		ros::spinOnce();
	}
	ROS_INFO("First position message recieved!");

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