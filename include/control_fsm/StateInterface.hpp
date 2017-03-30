#ifndef STATE_INTERFACE_HPP
#define STATE_INTERFACE_HPP

#include "EventData.hpp"
#include <iostream>
#include <mavros_msgs/PositionTarget.h>
#include "setpoint_msg_defines.h"
/*
NOTE:
FSM is not async so do not run any blocking code 
in any of these methods.
EventData is passed by reference and is NOT guaranteed to remain in scope. 
DO NOT store event data by reference
*/
class ControlFSM;
class StateInterface {
protected:
	mavros_msgs::PositionTarget _setpoint;
public:
	//Virtual destructor - override if needed
	virtual ~StateInterface() {}
	//Handles incoming external events
	virtual void handleEvent(ControlFSM& fsm, const EventData& event) = 0;
	//The function statebegin() will run on current state AFTER event/transition.
	//stateBegin is only implemented if needed by state.
	virtual void stateBegin(ControlFSM& fsm, const EventData& event) {}
	//Runs state specific code
	//loopState is only implemented if needed by state
	virtual void loopState(ControlFSM& fsm) {}
	//Should return name of the state - used for debugging purposes
	virtual std::string getStateName() const = 0;
	//Each state is responsible for delivering setpoints when the state is active. 
	//Be aware - it's return by const pointer - only return address of _setpoint.
	virtual const mavros_msgs::PositionTarget* getSetpoint() = 0;
};

#endif
