#pragma once

#include "StateInterface.hpp"

class BeginState : public StateInterface {
public:
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	std::string getStateName() const override { return "Begin"; }
	const mavros_msgs::PositionTarget& getSetpoint() override;
};
