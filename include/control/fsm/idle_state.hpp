#ifndef IDLE_STATE_HPP
#define IDLE_STATE_HPP

#include <iostream>
#include "state_interface.hpp"
#include <mavros_msgs/PositionTarget.h>
#include "event_data.hpp"

///Idle
class IdleState : public StateInterface {
public:
    IdleState();
    void handleEvent(ControlFSM& fsm, const EventData& event) override;
    std::string getStateName() const override { return "Idle"; }
    ascend_msgs::ControlFSMState getStateMsg() const override; 
    const mavros_msgs::PositionTarget* getSetpointPtr() override;
    void handleManual(ControlFSM &fsm) override;
    void stateInit(ControlFSM& fsm) override;
    void loopState(ControlFSM& fsm) override;
    void stateBegin(ControlFSM& fsm, const EventData& event) override;
};

#endif
