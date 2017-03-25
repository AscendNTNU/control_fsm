#pragma once

#include "StateInterface.hpp"

class BeginState : public StateInterface {
public:
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	void stateBegin(ControlFSM& fsm, const EventData& event) override;
	std::string getStateName() override { return "BeginState"; }
};
