#ifndef POSITION_HOLD_STATE_HPP
#define POSITION_HOLD_STATE_HPP
#include "StateInterface.hpp"

class PositionHoldState : public StateInterface {
private:
	double _posX, _posY, _posZ;
public:
	PositionHoldState();
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	void stateBegin(ControlFSM& fsm, const EventData& event) override;
	std::string getStateName() const override { return "Position hold"; }
	const mavros_msgs::PositionTarget* getSetpoint() override; 
};

#endif