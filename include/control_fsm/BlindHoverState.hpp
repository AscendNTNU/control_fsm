#ifndef BLIND_HOVER_STATE_HPP
#define BLIND_HOVER_STATE_HPP
#include "StateInterface.hpp"

class BlindHoverState : public StateInterface {
private:
	double _hoverAltitude = 0.5; //Pretty safe default value
public: 
	BlindHoverState();
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	void stateBegin(ControlFSM& fsm, const EventData& event) override;
	void loopState(ControlFSM& fsm) override;
	std::string getStateName() const override { return "Blind hover";}
	const mavros_msgs::PositionTarget* getSetpoint() override;
};

#endif