#ifndef PREFLIGHT_STATE_HPP
#define PREFLIGHT_STATE_HPP

#include "StateInterface.hpp"

///Runs preflight checks and transition to idle when ready
class PreFlightState : public StateInterface {
public:
    PreFlightState();
    void handleEvent(ControlFSM& fsm, const EventData& event) override;
    void stateBegin(ControlFSM& fsm, const EventData& event) override;
    std::string getStateName() const override { return "Preflight"; }
    //Returns setpoint
    const mavros_msgs::PositionTarget* getSetpoint() override;
};

#endif
