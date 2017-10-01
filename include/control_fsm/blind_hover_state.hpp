#ifndef BLIND_HOVER_STATE_HPP
#define BLIND_HOVER_STATE_HPP
#include "state_interface.hpp"

///State handling blind hover
class BlindHoverState : public StateInterface {
private:
    event_data _cmd;
public:
    BlindHoverState();
    void handleEvent(ControlFSM& fsm, const event_data& event) override;
    void stateBegin(ControlFSM& fsm, const event_data& event) override;
    void loopState(ControlFSM& fsm) override;
    std::string getStateName() const override { return "Blind hover";}
    const mavros_msgs::PositionTarget* getSetpoint() override;
    void handleManual(ControlFSM &fsm) override;
};

#endif