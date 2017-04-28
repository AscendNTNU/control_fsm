#include <control_fsm/ActionServer.hpp>
#include <control_fsm/EventData.hpp>
#include <control_fsm/ControlFSM.hpp>
#include <iostream>
ActionServer::ActionServer(ControlFSM* pFsm) : as_(nh_, "controlNodeActionServer", false) {
	this->pFsm_ = pFsm;
	as_.registerGoalCallback(boost::bind(&ActionServer::goalCB, this));
	as_.registerPreemptCallback(boost::bind(&ActionServer::preemptCB, this));
	as_.start();
}

void ActionServer::goalCB() {
	//Check if an action is already running
	if(actionIsRunning_) {
		RequestEvent abortEvent(RequestType::ABORT);
		pFsm_->handleEvent(abortEvent);
		if(actionIsRunning_) {
			ROS_WARN("[Control Action Server] CMD not stopping after abort - bug!");
		}
	}

	auto goal = *(as_.acceptNewGoal());
	switch(goal.goalType) {
		case goal.GOTOXYZ:
			startGoTo(goal);
			break;
		case goal.LANDXY:
			startLandXY(goal);
			break;
		/* case goal.LANDGB:
			startLandGB(goal);
			break; */
		default:
			ROS_ERROR("[Control Action Server] Not a valid or available action");
			as_.setPreempted();
			break;
	}
}
//Preempt action and send abort to fsm
void ActionServer::preemptCB() {
	if(actionIsRunning_) {
		RequestEvent abortEvent(RequestType::ABORT);
		pFsm_->handleEvent(abortEvent);
		if(actionIsRunning_) {
			ROS_WARN("[Control Action Server] CMD not properly terminated on abort - bug!");
		}
	}
	as_.setPreempted();
}

//If goal is goto, send valid goto cmd to fsm
void ActionServer::startGoTo(const ascend_msgs::ControlFSMGoal& goal) {
	GoToXYZCMDEvent goToEvent(goal.x, goal.y, goal.y, goal.yaw);
	goToEvent.setOnCompleteCallback([this]() {
		ascend_msgs::ControlFSMResult result;
		result.finished = true;
		actionIsRunning_ = false;
		as_.setSucceeded(result);
	});

	goToEvent.setOnErrorCallback([this](std::string msg) {
		ROS_WARN("[Control ActionServer] CMD error: %s", msg.c_str());
		ascend_msgs::ControlFSMResult result;
		result.finished = false;
		actionIsRunning_ = false;
		as_.setAborted(result);
	});
	pFsm_->handleEvent(goToEvent);
}
//If goal is landxy, send valid landxy cmd to fsm
void ActionServer::startLandXY(const ascend_msgs::ControlFSMGoal& goal) {
	LandXYCMDEvent landXYEvent(goal.x, goal.y, goal.yaw);
	landXYEvent.setOnCompleteCallback([this]() {
		ascend_msgs::ControlFSMResult result;
		result.finished = true;
		actionIsRunning_ = false;
		as_.setSucceeded(result);
	});
	landXYEvent.setOnErrorCallback([this](std::string msg) {
		ROS_WARN("[Control ActionServer] CMD error: %s", msg.c_str());
		ascend_msgs::ControlFSMResult result;
		result.finished = false;
		actionIsRunning_ = false;
		as_.setAborted(result);
	});
	pFsm_->handleEvent(landXYEvent);
}

void ActionServer::startLandGB(const ascend_msgs::ControlFSMGoal& goal) {
	//TODO Implement when landgb procedure is decided
	ROS_WARN("[Control ActionServer] LandGB not implemented!!");
	ascend_msgs::ControlFSMResult result;
	result.finished = false;
	actionIsRunning_ = false;
	as_.setAborted(result);
}

