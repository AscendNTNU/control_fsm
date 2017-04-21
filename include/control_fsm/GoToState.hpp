#ifndef GO_TO_STATE_HPP
#define GO_TO_STATE_HPP
#include "StateInterface.hpp"
#include <ros/ros.h>
#include <ascend_msgs/PathPlannerPoint.h>

class GoToState : public StateInterface {
private:
	EventData _cmd;
	ros::NodeHandle _nh;
	ros::Publisher _posPub;
	ros::Publisher _targetPub;

	struct {
		ascend_msgs::PathPlannerPoint plan;
		bool valid = false;
	} _currentPlan;

	void pathRecievedCB(const ascend_msgs::PathPlannerPoint& msg);
public:
	GoToState();
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	void stateBegin(ControlFSM& fsm, const EventData& event) override;
	void loopState(ControlFSM& fsm) override;
	std::string getStateName() const { return "GoTo";}
	const mavros_msgs::PositionTarget* getSetpoint();
};

#endif