#ifndef STATE_INTERFACE_HPP
#define STATE_INTERFACE_HPP

/** @page fsm_description FSM design
 *  @brief Describes the design pattern
 *  
 *  All states inherits the StateInterface class, and the
 *  FSM use polymorphism to switch between different states. 
 *  
 *  On transition between states, the StateInterface::stateBegin method will be called in the new state. 
 *  The active states StateInterface::loopState method will always run in a loop.
 *  The StateInterface::stateEnd method will be called before the fsm transition to another state.
 *  
 *  Transitioning between states is done with the ControlFSM::transtionTo method. 
 *  
 *  FSM is not async so do not run any blocking code 
 *  in any of these methods.
 *  EventData is passed by reference and is NOT guaranteed to remain in scope.
 *  DO NOT store event data by reference.
 *  States should handle all exceptions.
 *  Unhandled exceptions will be catched by try-catch in fsm loop, but it can lead to
 *  undefined behaviour. All state methods should gurantee nothrow!
 */

#include "event_data.hpp"
#include <iostream>
#include <mavros_msgs/PositionTarget.h>
#include "control/tools/setpoint_msg_defines.h"
#include <ascend_msgs/ControlFSMState.h>
#include <list>

class ControlFSM;

///Abstract interface class inherited by all states
/*
NOTE:
FSM is not async so do not run any blocking code 
in any of these methods.
EventData is passed by reference and is NOT guaranteed to remain in scope.
DO NOT store event data by reference.
States should handle all exceptions.
Unhandled exceptions will be catched by try-catch in fsm loop, but it can lead to
undefined behaviour. All state methods should gurantee nothrow!
*/
class StateInterface;
class StateInterface {
private:
    ///Flag used to check if state is ready - should be set by state init
    bool is_ready_ = false;

    /**
     * get vector of all instianciated states
     * This idiom is a workaround for the "static initialiazation fiasco"
     * @return vector of instanciated states
     */
    static std::list<StateInterface*>* getAllStatesVector();

    ///Assigmnet operator should be removed
    StateInterface& operator=(const StateInterface&) = delete;
protected:
    ///Mark state as ready
    void setStateIsReady(){ is_ready_ = true; }

    ///All states needs to return a valid setpoint
    mavros_msgs::PositionTarget setpoint_;
public:

    ///Constructor
    StateInterface() {  getAllStatesVector()->push_back(this); }

    ///States should never be copied
    StateInterface(const StateInterface&) = delete;

    ///Used to check if state is ready for flight
    bool stateIsReady() const { return is_ready_; }

    ///Used for state setup - remember to call setStateIsReady if overriding
    virtual void stateInit(ControlFSM& fsm) { setStateIsReady(); }

    ///Virtual destructor - override if needed
    virtual ~StateInterface() {}

    ///Handles incoming external events
    virtual void handleEvent(ControlFSM& fsm, const EventData& event) = 0;

    ///Handles loss of offboard mode - must be implemented by state
    virtual void handleManual(ControlFSM& fsm) = 0;
     
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
    
    ///Get state message
    virtual ascend_msgs::ControlFSMState getStateMsg() const = 0; 
    
    ///Returning a valid setpoint from state 
    /**Be aware - it's returned by const pointer - only return address of _setpoint.*/
    virtual const mavros_msgs::PositionTarget* getSetpointPtr() = 0;

    ///Static interface returning iterator to first state
    static std::list<StateInterface*>::const_iterator cbegin() { return getAllStatesVector()->cbegin(); }
    
    ///Static interface returning iterator to last + 1 state
    static std::list<StateInterface*>::const_iterator cend() { return getAllStatesVector()->cend(); }
  
    ///Returns number of instanciated states
    static size_t getNumStates() { return getAllStatesVector()->size(); }
};

#endif
