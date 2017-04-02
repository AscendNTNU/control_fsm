#include "control_fsm/ControlFSM.hpp"
#include "control_fsm/EventData.hpp"
#include <ros/ros.h>

//This is just for testing FSM transitions

void printOptions();
geometry_msgs::PoseStamped fakePosition();

int main(int argc, char** argv) {

	ros::init(argc, argv, "control_fsm_test");
	EventData event; //This object will be passed as a reference to the current state
	ControlFSM fsm; //Statemachine object
	std::cout << "Current state: " << fsm.getState()->getStateName() << std::endl;
	while(ros::ok()) {
		printOptions();
		int selection = -1;
		std::cout << "Your selection: ";
		std::cin >> selection;
		EventData event;
		switch(selection) {
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
				break;
			case 18:
				event.eventType = EventType::COMMAND;
				event.commandType = CommandType::LANDXY;
				event.setOnCompleteCallback([](){
					std::cout << "Command complete!" << std::endl;
				});
				break;
			case 19:
				event.eventType = EventType::COMMAND;
				event.commandType = CommandType::GOTOXYZ;
				event.setOnCompleteCallback([](){
					std::cout << "Command complete!" << std::endl;
				});
				break;
			case 20:
				event.eventType = EventType::COMMAND;
				event.commandType = CommandType::LANDGB;
				event.setOnCompleteCallback([](){
					std::cout << "Command complete!" << std::endl;
				});
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
			case 26:
				fsm.setPosition(fakePosition());
				break;
			default:
				std::cout << "Not an option" << std::endl;
				break;
		}
		fsm.handleEvent(event);
		fsm.loopCurrentState();
	}
}

void printOptions() {
	std::cout << "Requests:\n";
	std::cout << "1: NONE\n";
	std::cout << "2: ABORT\n";
	std::cout << "3: BEGIN\n";
	std::cout << "4: END\n";
	std::cout << "5: PREFLIGHT\n";
	std::cout << "6: IDLE\n";
	std::cout << "7: SHUTDOWN\n";
	std::cout << "8: TAKEOFF\n";
	std::cout << "9: BLINDHOVER\n";
	std::cout << "10: POSHOLD\n";
	std::cout << "11: GOTO\n";
	std::cout << "12: LAND\n";
	std::cout << "13: BLINDLAND\n";
	std::cout << "14: TRACKGB\n";
	std::cout << "15: INTERGB\n";
	std::cout << "16: ESTIMATORADJ\n";
	std::cout << "\nCommands:\n";
	std::cout << "17: NONE\n";
	std::cout << "18: LANDXY\n";
	std::cout << "19: GOTOXYZ\n";
	std::cout << "20: LANDGB\n";

	std::cout << "\nOther event types:\n";
	std::cout << "21: ARMED\n";
	std::cout << "22: DISARMED\n";
	std::cout << "23: POSREGAINED\n";
	std::cout << "24: POSLOST\n";
	std::cout << "25: GROUNDDETECTED\n";
	std::cout << "26: Fake position XYZ\n";
	std::cout << std::flush;
}

geometry_msgs::PoseStamped fakePosition() {
	double x, y, z;
	std::cout << "Enter X: ";
	std::cin >> x;
	std::cout << "Enter Y: ";
	std::cin >> y;
	std::cout << "Enter Z: ";
	std::cin >> z;
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = x;
	pose.pose.position.y = y;
	pose.pose.position.z = z;
	return pose;
}
