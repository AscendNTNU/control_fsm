#ifndef LAND_STATE_HPP
#define LAND_STATE_HPP
#include "state_interface.hpp"

///Lands the drone
class LandState : public StateInterface {
private:
    event_data _cmd;
public:
    LandState();
    void handleEvent(ControlFSM& fsm, const event_data& event) override;
    void stateBegin(ControlFSM& fsm, const event_data& event) override;
    void loopState(ControlFSM& fsm);
    std::string getStateName() const override { return "Land"; }
    const mavros_msgs::PositionTarget* getSetpoint();
    void handleManual(ControlFSM &fsm) override;
};
#endif