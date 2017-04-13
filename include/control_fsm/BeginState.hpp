#ifndef BEGIN_STATE_HPP
#define BEGIN_STATE_HPP

#include "StateInterface.hpp"

class BeginState : public StateInterface {
public:
	BeginState();
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	std::string getStateName() const override { return "Begin"; }
	const mavros_msgs::PositionTarget* getSetpoint() override;
};

#endif
