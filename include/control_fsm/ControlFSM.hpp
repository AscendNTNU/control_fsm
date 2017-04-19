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

///Main FSM logic
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
	
	//Static instances of the different states
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

	///Holds a pointer to current running state
	/**Struct "vault" explanation:
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

	///Current drone position
	struct {
		bool validXY = false;
		geometry_msgs::PoseStamped position;
	} _dronePosition;
	
	///Is drone in an active state?
	bool _isActive = false;

protected:
	///States can only be changed by states or FSM logic
	void transitionTo(StateInterface& state, StateInterface* _pCaller, const EventData& event);
	
public:
	
	///Constructor sets default/starting state
	ControlFSM() { _stateVault._pCurrentState = &BEGINSTATE; }
	
	///Destructor not used to anything specific.
	~ControlFSM() {}

	///Get pointer to the current running state
	StateInterface* getState() { return _stateVault._pCurrentState; }
	
	///Handles incoming event
	/**Sends the external event to the current running state.
	 The event will be handled by the handleState function in the state interface.*/
	void handleEvent(const EventData& event);
	
	///Loops the current running state
	void loopCurrentState(void);
	
	///Send errormessage to user via ROS_ERROR
	void handleFSMError(std::string errMsg);
	
	///Send info message to user via ROS_INFO
	void handleFSMInfo(std::string infoMsg);
	
	///Send warning message to user via ROS_WARN
	void handleFSMWarn(std::string warnMsg);
	
	///Send debug message to user via ROS_DEBUG
	void handleFSMDebug(std::string debugMsg);
	
	///Returns setpoint from current state
	const mavros_msgs::PositionTarget* getSetpoint() { return getState()->getSetpoint(); }
	
	///Set current drone position
	void setPosition(const geometry_msgs::PoseStamped& pose);
	
	///Get current position - will return nullptr if invalid
	const geometry_msgs::PoseStamped* getPositionXYZ() {return _dronePosition.validXY ? &_dronePosition.position : nullptr; }
	
	/// \deprecated Get altitude (should always be correct - 1D lidar)
	double getPositionZ() { return _dronePosition.position.pose.position.z; }
	
	///Checks if FSM is in an "active" state
	bool getIsActive() { return _isActive; }
};

#endif

