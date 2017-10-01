#ifndef PREFLIGHT_STATE_HPP
#define PREFLIGHT_STATE_HPP

#include "state_interface.hpp"

///Runs preflight checks and transition to idle when ready
class PreFlightState : public StateInterface {
public:
    PreFlightState();
    void handleEvent(ControlFSM& fsm, const event_data& event) override;
    void stateBegin(ControlFSM& fsm, const event_data& event) override;
    std::string getStateName() const override { return "Preflight"; }
    //Returns setpoint
    const mavros_msgs::PositionTarget* getSetpoint() override;
    void handleManual(ControlFSM &fsm) override;
};

#endif
