#ifndef CONTROL_FSM_HPP
#define CONTROL_FSM_HPP

#include "StateInterface.hpp"
#include "BeginState.hpp"
#include "PreFlightState.hpp"
#include "IdleState.hpp"
#include "TakeoffState.hpp"
#include "BlindHoverState.hpp"
#include "PositionHoldState.hpp"

class ControlFSM {
private:
	//Add state classes as friend classes here - allowing them to use transitionTo.
	friend class BeginState;
	friend class PreFlightState;
	friend class IdleState;
	friend class TakeoffState;
	friend class BlindHoverState;
	friend class PositionHoldState;
	//Add static instances of the different desired states here!
	static BeginState BEGINSTATE;
	static PreFlightState PREFLIGHTSTATE;
	static IdleState IDLESTATE;
	static TakeoffState TAKEOFFSTATE;
	static BlindHoverState BLINDHOVERSTATE;
	static PositionHoldState POSITIONHOLDSTATE;
	//Only this ControlFSM class should be allowed to access the _pCurrentState pointer.
	/*Struct explanation:
	The struct (with instance _stateHolder) keeps the _pCurrentState private. 
	The struct friends the FSM class so the FSM class can access the pointer. 
	Even though other classes or function might have access to FSM private variables through friend,
	they still wont have access to the pointer.
	*/
	struct {
	friend class ControlFSM;
	private:
		StateInterface* _pCurrentState = nullptr; //This need to be set to a start state in constructor
	} _stateVault;

protected:
	//States can only be changed by states or FSM logic
	void transitionTo(StateInterface& state, StateInterface* _pCaller, const EventData& event);
	
public:
	//Constructor sets default/starting state
	ControlFSM() { _stateVault._pCurrentState = &BEGINSTATE; }
	//Destructor not used.
	~ControlFSM() {}
	//Get a pointer to the current running state
	StateInterface* getState() { return _stateVault._pCurrentState; }
	//Sends an external event to the current running state
	//Note the use of unique_ptr: handleEvent will own pEventData object and it will be destroyed after use.
	void handleEvent(const EventData& event);
	//Loops the current state (via the loopState function in state interface)
	void loopCurrentState(void);
	//Send errormessage to user
	void handleFSMError(std::string errMsg);
	//Send info message to user
	void handleFSMInfo(std::string warnMsg);
};

#endif

