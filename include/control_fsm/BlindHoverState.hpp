#ifndef BLIND_HOVER_STATE_HPP
#define BLIND_HOVER_STATE_HPP
#include "StateInterface.hpp"

///State handling blind hover
class BlindHoverState : public StateInterface {
private:
	EventData _cmd;
public: 
	BlindHoverState();
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	void stateBegin(ControlFSM& fsm, const EventData& event) override;
	void loopState(ControlFSM& fsm) override;
	std::string getStateName() const override { return "Blind hover";}
	const mavros_msgs::PositionTarget* getSetpoint() override;

	void handleManual(ControlFSM &fsm) override;
};

#endif