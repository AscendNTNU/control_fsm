#pragma once

#include "StateInterface.hpp"

class PreFlightState : public StateInterface {
public:
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	void stateBegin(ControlFSM& fsm, const EventData& event) override;
    std::string getStateName() const override { return "PreFlightState"; }
    //Returns setpoint
	const mavros_msgs::PositionTarget& getSetpoint() const override;
};
