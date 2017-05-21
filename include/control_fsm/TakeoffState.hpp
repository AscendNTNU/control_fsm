#ifndef TAKE_OFF_STATE_HPP
#define TAKE_OFF_STATE_HPP
#include "StateInterface.hpp"
#include <ros/ros.h>

#define DEFAULT_TAKEOFF_ALTITUDE 1.0f 
#define DEFAULT_TAKEOFF_ALTITUDE_REACHED_MARGIN 0.1

///Takeoff state
class TakeoffState : public StateInterface {
private:
	EventData _cmd;
	double _altitude_reached_margin = DEFAULT_TAKEOFF_ALTITUDE_REACHED_MARGIN;
public:
	TakeoffState();
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	void stateBegin(ControlFSM& fsm, const EventData& event) override;
	void loopState(ControlFSM& fsm) override;
	std::string getStateName() const override { return "Takeoff";}
	const mavros_msgs::PositionTarget* getSetpoint() override;

	void abort(ControlFSM &fsm) override;

	void handleCMD(ControlFSM &fsm, const EventData &event) override;
};


#endif
