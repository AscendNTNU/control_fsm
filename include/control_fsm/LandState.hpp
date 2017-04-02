#ifndef LAND_STATE_HPP
#define LAND_STATE_HPP
#include "StateInterface.hpp"

class LandState : public StateInterface {
private:
	EventData _cmd;
public:
	LandState();
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	void stateBegin(ControlFSM& fsm, const EventData& event) override;
	void loopState(ControlFSM& fsm);
	std::string getStateName() const override { return "Land"; }
	const mavros_msgs::PositionTarget* getSetpoint();
};
#endif