#include <iostream>
#include <std_msgs/Int32.h>
#include <ros/ros.h>

void printOptions();

int main(int argc, char** argv) {
	ros::init(argc, argv, "control_fsm_gazebo_input");
	ros::NodeHandle n;

	ros::Publisher input_pub = n.advertise<std_msgs::Int32>("control_fsm_gazebo_input/value", 10);

	while(ros::ok()) {
		printOptions();
		int selection = -1;
		std::cout << "Your selection: ";
		std::cin >> selection;
		std::cout << "\n" << std::endl;
		std_msgs::Int32 val;
		val.data = selection;
		input_pub.publish(val);
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
	std::cout << std::flush;
}