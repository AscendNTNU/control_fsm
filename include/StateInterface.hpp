#pragma once
#include "EventData.hpp"
#include <iostream>
/*
NOTE:
FSM is not async so do not run any blocking code 
in any of these methods.
*/
class ControlFSM;
class StateInterface {
public:
	//Virtual destructor - override if needed
	virtual ~StateInterface() {}
	//Handles incoming external events
	virtual void handleEvent(ControlFSM& fsm, const EventData& event) = 0;
	//The function statebegin() wull run on current state AFTER event/transition.
	virtual void stateBegin(ControlFSM& fsm, const EventData& event) = 0;
	//Runs state specific code
	virtual void loopState(ControlFSM& fsm) = 0;
	//Should return name of the state - used for debugging purposes
	virtual std::string getStateName() = 0;


	/*
	TODO: Add method for returning a valid setpoint for MAVROS
	*/
};
