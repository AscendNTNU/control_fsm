#ifndef PREFLIGHT_STATE_HPP
#define PREFLIGHT_STATE_HPP

#include "state_interface.hpp"

///Runs preflight checks and transition to idle when ready
class PreFlightState : public StateInterface {
public:
    PreFlightState();
    void handleEvent(ControlFSM& fsm, const EventData& event) override;
    void stateBegin(ControlFSM& fsm, const EventData& event) override;
    std::string getStateName() const override { return "Preflight"; }
    ascend_msgs::ControlFSMState getStateMsg() override;
    //Returns setpoint
    const mavros_msgs::PositionTarget* getSetpointPtr() override;
    void handleManual(ControlFSM &fsm) override;
};

#endif
