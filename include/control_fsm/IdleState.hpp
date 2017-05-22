#ifndef IDLE_STATE_HPP
#define IDLE_STATE_HPP

#include <iostream>
#include "StateInterface.hpp"
#include <mavros_msgs/PositionTarget.h>
#include "EventData.hpp"

///Idle
class IdleState : public StateInterface {
public:
	IdleState();
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	std::string getStateName() const { return "Idle"; }
	const mavros_msgs::PositionTarget* getSetpoint();

	void handleAbort(ControlFSM &fsm) override;

	void handleCMD(ControlFSM &fsm, const EventData &event) override;
};

#endif
