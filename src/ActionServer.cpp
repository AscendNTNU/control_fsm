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
	if(actionIsRunning_) {
		RequestEvent abortEvent(RequestType::ABORT);
		pFsm_->handleEvent(abortEvent);
		if(actionIsRunning_) {
			ROS_WARN("ACTION SERVER: CMD not stopping after abort - bug!");
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
		case goal.LANDGB:
			startLandGB(goal);
			break;
		default:
			ROS_ERROR("ACTION SERVER: Not a valid action");
			as_.setPreempted();
			break;
	}
}

void ActionServer::preemptCB() {
	if(actionIsRunning_) {
		RequestEvent abortEvent(RequestType::ABORT);
		pFsm_->handleEvent(abortEvent);
		if(actionIsRunning_) {
			ROS_WARN("ACTION SERVER: CMD not properly terminated on abort - bug!");
		}
	}
	as_.setPreempted();
}

void ActionServer::startGoTo(const control_fsm::ControlNodeGoal& goal) {
	GoToXYZCMDEvent goToEvent(goal.x, goal.y, goal.y, goal.yaw);
	goToEvent.setOnCompleteCallback([this]() {
		control_fsm::ControlNodeResult result;
		result.finished = true;
		actionIsRunning_ = false;
		as_.setSucceeded(result);
	});

	goToEvent.setOnErrorCallback([this](std::string msg) {
		ROS_WARN("ACTION SERVER ERROR CALLBACK: %s", msg.c_str());
		control_fsm::ControlNodeResult result;
		result.finished = false;
		actionIsRunning_ = false;
		as_.setAborted(result);
	});
	pFsm_->handleEvent(goToEvent);
}

void ActionServer::startLandXY(const control_fsm::ControlNodeGoal& goal) {
	LandXYCMDEvent landXYEvent(goal.x, goal.y, goal.yaw);
	landXYEvent.setOnCompleteCallback([this]() {
		control_fsm::ControlNodeResult result;
		result.finished = true;
		actionIsRunning_ = false;
		as_.setSucceeded(result);
	});
	landXYEvent.setOnErrorCallback([this](std::string msg) {
		ROS_WARN("ACTION SERVER ERROR CALLBACK: %s", msg.c_str());
		control_fsm::ControlNodeResult result;
		result.finished = false;
		actionIsRunning_ = false;
		as_.setAborted(result);
	});
	pFsm_->handleEvent(landXYEvent);
}

void ActionServer::startLandGB(const control_fsm::ControlNodeGoal& goal) {
	//TODO Implement when landgb procedure is decided
}

