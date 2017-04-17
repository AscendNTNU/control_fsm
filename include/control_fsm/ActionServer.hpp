#ifndef ACTION_SERVER_HPP
#define ACTION_SERVER_HPP
#include <actionlib/server/simple_action_server.h>
#include <control_fsm/ControlFSM.hpp>
#include <control_fsm/ControlNodeAction.h>
#include <ros/ros.h>

class ActionServer {
private:
	ros::NodeHandle nh_;
	bool actionIsRunning_ = false;
	ControlFSM* pFsm_ = nullptr;
	actionlib::SimpleActionServer<control_fsm::ControlNodeAction> as_;
	void goalCB();
	void preemptCB();
	void startGoTo(const control_fsm::ControlNodeGoal& goal);
	void startLandXY(const control_fsm::ControlNodeGoal& goal);
	void startLandGB(const control_fsm::ControlNodeGoal& goal);
public:
	ActionServer(ControlFSM* pFsm);
};

#endif