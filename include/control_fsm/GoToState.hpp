#ifndef GO_TO_STATE_HPP
#define GO_TO_STATE_HPP
#include "StateInterface.hpp"
#include <ros/ros.h>
#include <ascend_msgs/PathPlannerPlan.h>

#define DEFAULT_DEST_REACHED 0.3

///Moves drone to XYZ 
class GoToState : public StateInterface {
private:
	EventData _cmd;
	ros::NodeHandle _nh;
	ros::Publisher _posPub;
	ros::Publisher _targetPub;
	bool _isActive = false;
	struct {
		ascend_msgs::PathPlannerPlan plan;
		bool valid = false;
	} _currentPlan;

	float _dest_reached_margin = DEFAULT_DEST_REACHED;

	void pathRecievedCB(const ascend_msgs::PathPlannerPlan& msg);
public:
	GoToState();
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	void stateBegin(ControlFSM& fsm, const EventData& event) override;
	void loopState(ControlFSM& fsm) override;
	void stateEnd(ControlFSM& fsm, const EventData& event) override;
	std::string getStateName() const { return "GoTo";}
	const mavros_msgs::PositionTarget* getSetpoint();
};

#endif