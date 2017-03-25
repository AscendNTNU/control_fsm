#pragma once
#include "EventData.hpp"
#include <iostream>
#include <mavros_msgs/PositionTarget.h>
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
	//The function statebegin() will run on current state AFTER event/transition.
	//stateBegin is only implemented if needed by state.
	virtual void stateBegin(ControlFSM& fsm, const EventData& event) {}
	//Runs state specific code
	//loopState is only implemented if needed by state
	virtual void loopState(ControlFSM& fsm) {}
	//Should return name of the state - used for debugging purposes
	virtual std::string getStateName() = 0;
	//Each state is responsible for delivering setpoints when the state is active. 
	virtual const mavros_msgs::PositionTarget& getSetPoint() = 0;


	/*
	TODO: Add method for returning a valid setpoint for MAVROS
	*/
};
