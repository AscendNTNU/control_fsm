#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_fsm/ControlNodeAction.h>

void getValidXYYaw(float*, float*, float*);
void getValidXYZYaw(float*, float*, float*, float*);
void goalStartedCB();
void goalDoneCB(const actionlib::SimpleClientGoalState& state, const control_fsm::ControlNodeResultConstPtr& result);
void goalFeedbackCB(const control_fsm::ControlNodeFeedbackConstPtr& feedback);
control_fsm::ControlNodeGoal getValidGoal();
bool activeGoal = false;
int main(int argc, char** argv) {
	ros::init(argc, argv, "control_fsm_actionlib_client");
	ros::AsyncSpinner spinner(2);
	spinner.start();
	actionlib::SimpleActionClient<control_fsm::ControlNodeAction> client("controlNodeActionServer", false);
	ROS_INFO("Waiting for server!");
	client.waitForServer();
	ROS_INFO("Server started, sending goals");
	while(ros::ok()) {
		control_fsm::ControlNodeGoal goal = getValidGoal();
		if(goal.goalType == -1) {
			client.cancelGoal();

		} else {
			client.sendGoal(goal, &goalDoneCB, &goalStartedCB, &goalFeedbackCB);
		}
	}

}

control_fsm::ControlNodeGoal getValidGoal() {
	std::cout << "Options: " << "\n";
	std::cout << "1: GOTOXYZ" << "\n";
	std::cout << "2: LANDXY" << "\n";
	std::cout << "3: LANDGB" << "\n";
	std::cout << "4: ABORT" << "\n";
	std::cout << "Please select: " << std::flush;
	control_fsm::ControlNodeGoal goal;
	int opt;
	std::cin >> opt;
	switch(opt) {
		case 1:
			goal.goalType = goal.GOTOXYZ;
			getValidXYZYaw(&goal.x, &goal.y, &goal.z, &goal.yaw);
			break;
		case 2:
			goal.goalType = goal.LANDXY;
			getValidXYYaw(&goal.x, &goal.y, &goal.yaw);
			break;
		case 3:
			goal.goalType = goal.LANDGB;
			std::cout << "GB nr: ";
			std::cin >> goal.gbNr;
			break;
		case 4:
			goal.goalType = -1; //Hacky way to signal abort
			break;
		default:
			std::cout << "Not an option" << std::endl;
			return getValidGoal();
	}
	return goal;


}

void getValidXYYaw(float* x, float* y, float* yaw) {
	std::cout << "Enter X: ";
	std::cin >> *x;
	std::cout << "Enter Y: "; 
	std::cin >> *y;
	std::cout << "Enter Yaw: ";
	std::cin >> *yaw;
}
void getValidXYZYaw(float* x, float* y, float* z, float* yaw) {
	getValidXYYaw(x, y, yaw);
	std::cout << "Enter Z: ";
	std::cin >> *z;
}


void goalStartedCB() {
	ROS_INFO("New goal started");
}
void goalDoneCB(const actionlib::SimpleClientGoalState& state, const control_fsm::ControlNodeResultConstPtr& result) {
	ROS_INFO("Goal finished: %s", state.toString().c_str());
	if(result->finished) {
		ROS_INFO("Goal completed!");
	} else {
		ROS_INFO("Goal failed!");
	}
}
void goalFeedbackCB(const control_fsm::ControlNodeFeedbackConstPtr& feedback) {}