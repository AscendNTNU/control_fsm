#ifndef IDLE_STATE_HPP
#define IDLE_STATE_HPP

#include <iostream>
#include "StateInterface.hpp"
#include <mavros_msgs/PositionTarget.h>
#include "EventData.hpp"

class IdleState : public StateInterface {
public:
	IdleState();
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	std::string getStateName() const { return "Idle"; }
	const mavros_msgs::PositionTarget* getSetpoint(); 
};

#endif
