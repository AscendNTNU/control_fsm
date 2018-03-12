#ifndef BLIND_HOVER_STATE_HPP
#define BLIND_HOVER_STATE_HPP
#include "state_interface.hpp"

///State handling blind hover
class BlindHoverState : public StateInterface {
private:
    EventData cmd_;
public:
    BlindHoverState();
    void handleEvent(ControlFSM& fsm, const EventData& event) override;
    void stateBegin(ControlFSM& fsm, const EventData& event) override;
    void loopState(ControlFSM& fsm) override;
    ascend_msgs::ControlFSMState getStateMsg() const override;
    std::string getStateName() const override { return "Blind hover";}
    const mavros_msgs::PositionTarget* getSetpointPtr() override;
    void handleManual(ControlFSM &fsm) override;
};

#endif
