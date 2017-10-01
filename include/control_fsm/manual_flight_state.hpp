#ifndef MANUAL_FLIGHT_STATE
#define MANUAL_FLIGHT_STATE

#include "state_interface.hpp"

///Runs preflight checks and transition to idle when ready
class ManualFlightState : public StateInterface {
public:
    ManualFlightState();
    void handleEvent(ControlFSM& fsm, const EventData& event) override;
    std::string getStateName() const override { return "ManualFlight"; }
    void loopState(ControlFSM& fsm) override;
    //Returns setpoint
    const mavros_msgs::PositionTarget* getSetpoint() override;
    void handleManual(ControlFSM &fsm) override;
};

#endif
