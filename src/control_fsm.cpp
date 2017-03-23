#include "ControlFSM.hpp"
#include "EventData.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {

	ros::init(argc, argv, "control_fsm_test");
	EventData event; //This object will be passed as a reference to the current state
	ControlFSM fsm; //Statemachine object
	std::cout << "Current state: " << fsm.getState()->getStateName() << std::endl;
	std::cout << "Request transition to preflight" << std::endl;
	event.request = RequestType::PREFLIGHT; //Request transition to preflight
	fsm.handleEvent(event); //Process request
	std::cout << "Request abort preflight" << std::endl;
	event.request = RequestType::ABORT; //Request abort of preflight
	fsm.handleEvent(event); //Request abort
	std::cout << "Request an illegal transition to land" << std::endl;
	event.request = RequestType::LAND;
	fsm.handleEvent(event);
}
