#ifndef PREFLIGHT_STATE_HPP
#define PREFLIGHT_STATE_HPP

#include "StateInterface.hpp"

class PreFlightState : public StateInterface {
public:
	PreFlightState();
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
    std::string getStateName() const override { return "Preflight"; }
    //Returns setpoint
	const mavros_msgs::PositionTarget* getSetpoint() override;
};

#endif
