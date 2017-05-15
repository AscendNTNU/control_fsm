#ifndef STATE_INTERFACE_HPP
#define STATE_INTERFACE_HPP

#include "EventData.hpp"
#include <iostream>
#include <mavros_msgs/PositionTarget.h>
#include "setpoint_msg_defines.h"

class ControlFSM;

///Abstract interface class inherited by all states
/*
NOTE:
FSM is not async so do not run any blocking code 
in any of these methods.
EventData is passed by reference and is NOT guaranteed to remain in scope. 
DO NOT store event data by reference
*/
class StateInterface {
protected:
	mavros_msgs::PositionTarget _setpoint;
public:

	//Consider adding stateInit method to function as a constructor for the static instances
	
	///Virtual destructor - override if needed
	virtual ~StateInterface() {}

	///Handles incoming external events
	virtual void handleEvent(ControlFSM& fsm, const EventData& event) = 0;
	
	///Runs on current state AFTER transition
	/**stateBegin is only implemented if needed by state.*/
	virtual void stateBegin(ControlFSM& fsm, const EventData& event) {}

	///Runs on current state BEFORE transition
	/**stateEnd is only implemented if needed by state*/
	virtual void stateEnd(ControlFSM& fsm, const EventData& event) {}
	
	///Runs state specific code
	/**loopState is only implemented if needed by state*/
	virtual void loopState(ControlFSM& fsm) {}
	
	///Should return name of the state - used for debugging purposes
	virtual std::string getStateName() const = 0;
	
	///Returning a valid setpoint from state 
	/**Be aware - it's return by const pointer - only return address of _setpoint.*/
	virtual const mavros_msgs::PositionTarget* getSetpoint() = 0;
};

#endif
