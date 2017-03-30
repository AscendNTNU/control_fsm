#ifndef SHUTDOWN_STATE_HPP
#define SHUTDOWN_STATE_HPP
#include "StateInterface.hpp"

class ShutdownState : public StateInterface {
public:
	ShutdownState();
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	void loopState(ControlFSM& fsm) override;
	std::string getStateName() const override { return "Shutdown"; }
	const mavros_msgs::PositionTarget* getSetpoint() override;
};

#endif