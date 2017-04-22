#ifndef POSITION_HOLD_STATE_HPP
#define POSITION_HOLD_STATE_HPP
#include "StateInterface.hpp"

///Holds the current position
class PositionHoldState : public StateInterface {
public:
	PositionHoldState();
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	void stateBegin(ControlFSM& fsm, const EventData& event) override;
	std::string getStateName() const override { return "Position hold"; }
	const mavros_msgs::PositionTarget* getSetpoint() override; 
};

#endif