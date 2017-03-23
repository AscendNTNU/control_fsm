#include "ControlFSM.hpp"
#include "EventData.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "control_fsm");
	EventData event;
	ControlFSM fsm;
	std::cout << "Current state: " << fsm.getState()->getStateName() << std::endl;
	event.request = RequestType::PREFLIGHT;
	std::cout << "Transition to preflight" << std::endl;
	fsm.handleEvent(event);
	std::cout << "Current state: " << fsm.getState()->getStateName() << std::endl;
	event.request = RequestType::ABORT;
	std::cout << "Abort preflight" << std::endl;
	fsm.handleEvent(event);
    std::cout << "Current state: " << fsm.getState()->getStateName() << std::endl;
	std::cout << "Illegal transition to land" << std::endl;
	event.request = RequestType::LAND;
	fsm.handleEvent(event);
	std::cout << "Current state: " << fsm.getState()->getStateName() << std::endl;
}
