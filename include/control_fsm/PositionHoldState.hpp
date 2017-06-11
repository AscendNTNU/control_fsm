#ifndef POSITION_HOLD_STATE_HPP
#define POSITION_HOLD_STATE_HPP
#include "StateInterface.hpp"
#include <memory>
#include <ros/ros.h>
#include <ascend_msgs/PointArray.h>
#define DEFAULT_SAFE_HOVER_ALT 2.5

///Holds the current position
class PositionHoldState : public StateInterface {
private:
	ros::Subscriber _lidarSub;
	bool _isActive = false;
	double _safeHoverAlt = DEFAULT_SAFE_HOVER_ALT;
	ControlFSM* _pFsm = nullptr;
public:
	PositionHoldState();
	void stateInit(ControlFSM& fsm) override;
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	void stateBegin(ControlFSM& fsm, const EventData& event) override;
	void stateEnd(ControlFSM& fsm, const EventData& eventData) override;
	std::string getStateName() const override { return "Position hold"; }
	const mavros_msgs::PositionTarget* getSetpoint() override;
	void obsCB(const ascend_msgs::PointArray::ConstPtr& msg);
	bool stateIsReady(ControlFSM &fsm) override;
};

#endif