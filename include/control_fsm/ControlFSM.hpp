#ifndef CONTROL_FSM_HPP
#define CONTROL_FSM_HPP

#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>

#include "StateInterface.hpp"
#include "BeginState.hpp"
#include "PreFlightState.hpp"
#include "IdleState.hpp"
#include "TakeoffState.hpp"
#include "BlindHoverState.hpp"
#include "PositionHoldState.hpp"
#include "ShutdownState.hpp"
#include "EstimateAdjustState.hpp"
#include "TrackGBState.hpp"
#include "InteractGBState.hpp"
#include "GoToState.hpp"
#include "LandState.hpp"
#include "BlindLandState.hpp"

class ControlFSM {
private:
	//Add state classes as friend classes here - allowing them to use transitionTo.
	friend class BeginState;
	friend class PreFlightState;
	friend class IdleState;
	friend class TakeoffState;
	friend class BlindHoverState;
	friend class PositionHoldState;
	friend class ShutdownState;
	friend class EstimateAdjustState;
	friend class TrackGBState;
	friend class InteractGBState;
	friend class GoToState;
	friend class LandState;
	friend class BlindLandState;
	//Add static instances of the different desired states here!
	static BeginState BEGINSTATE;
	static PreFlightState PREFLIGHTSTATE;
	static IdleState IDLESTATE;
	static TakeoffState TAKEOFFSTATE;
	static BlindHoverState BLINDHOVERSTATE;
	static PositionHoldState POSITIONHOLDSTATE;
	static ShutdownState SHUTDOWNSTATE;
	static EstimateAdjustState ESTIMATEADJUSTSTATE;
	static TrackGBState TRACKGBSTATE;
	static InteractGBState INTERACTGBSTATE;
	static GoToState GOTOSTATE;
	static LandState LANDSTATE;
	static BlindLandState BLINDLANDSTATE;

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

	struct {
		bool valid = false;
		geometry_msgs::PoseStamped position;
	} _dronePosition;
	

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
	void handleFSMInfo(std::string infoMsg);
	//Send warning message to user
	void handleFSMWarn(std::string warnMsg);
	//Send debug message to user
	void handleFSMDebug(std::string debugMsg);
	//Returns setpoint from current state
	const mavros_msgs::PositionTarget* getSetpoint() { return getState()->getSetpoint(); }
	//Set current position
	void setPosition(const geometry_msgs::PoseStamped& pose);
	//Get current position
	const geometry_msgs::PoseStamped* getPosition() {return _dronePosition.valid ? &_dronePosition.position : nullptr; }


};

#endif

