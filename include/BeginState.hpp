#pragma once

#include "StateInterface.hpp"

class BeginState : public StateInterface {
public:
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	void stateBegin(ControlFSM& fsm, const EventData& event) override;
	std::string getStateName() const override { return "BeginState"; }
	const mavros_msgs::PositionTarget& getSetpoint() const override;
};
