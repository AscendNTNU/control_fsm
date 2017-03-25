#pragma once

#include "StateInterface.hpp"

class PreFlightState : public StateInterface {
public:
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
    std::string getStateName() const override { return "Preflight"; }
    //Returns setpoint
	const mavros_msgs::PositionTarget& getSetpoint() override;
};
