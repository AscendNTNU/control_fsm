#include <ros/ros.h>

#include "control_fsm/ControlFSM.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>


bool first_position_recieved = false;
bool is_armed = false;
bool is_offboard = false;


ControlFSM fsm;

void local_pos_callback(const geometry_msgs::PoseStamped& input);
void state_changed_callback(const mavros_msgs::State& state);

int main(int argc, char** argv) {

	ros::init(argc, argv, "control_fsm_main");
	
	//Subscribe to neccesary topics
	ros::NodeHandle n;
	ros::Subscriber local_pos_sub = n.subscribe("mavros/mocap/pose", 10, local_pos_callback);
	
	ros::Publisher setpoint_pub = n.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

	ros::spinOnce();

	ros::Rate loop_rate(30);

	//Wait for all systems to initalize
	while(ros::ok && !first_position_recieved) {
		ros::spinOnce();
	}

	while(ros::ok()) {
		//TODO Take get input from planning or other 

		ros::spinOnce();
		fsm.loopCurrentState();
		const mavros_msgs::PositionTarget* pSetpoint = fsm.getSetpoint();
		setpoint_pub.publish(*pSetpoint);
		loop_rate.sleep();
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