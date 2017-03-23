#pragma once

#include "StateInterface.hpp"

class PreFlightState : public StateInterface {
public:
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	void stateBegin(ControlFSM& fsm, const EventData& event) override;
	void loopState(ControlFSM& fsm) override;
    std::string getStateName() override { return "PreFlightState"; }
};
