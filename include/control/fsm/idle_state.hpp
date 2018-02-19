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
    std::string getStateName() const { return "Idle"; }
    const mavros_msgs::PositionTarget* getSetpointPtr();
    void handleManual(ControlFSM &fsm) override;
};

#endif
