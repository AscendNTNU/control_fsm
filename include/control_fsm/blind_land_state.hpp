#ifndef BLIND_LAND_STATE_HPP
#define BLIND_LAND_STATE_HPP
#include "state_interface.hpp"

///State handling blind landings
class BlindLandState : public StateInterface {
public:
    BlindLandState();
    void handleEvent(ControlFSM& fsm, const event_data& event) override;
    void stateBegin(ControlFSM& fsm, const event_data& event) override;
    std::string getStateName() const { return "BlindLand";}
    const mavros_msgs::PositionTarget* getSetpoint();
    void handleManual(ControlFSM &fsm) override;
};
#endif