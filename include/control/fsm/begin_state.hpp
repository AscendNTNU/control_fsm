#ifndef BEGIN_STATE_HPP
#define BEGIN_STATE_HPP

#include "state_interface.hpp"

///Entrypoint state
class BeginState : public StateInterface {
public:
    BeginState();
    void handleEvent(ControlFSM& fsm, const EventData& event) override;
    std::string getStateName() const override { return "Begin"; }
    ascend_msgs::ControlFSMState getStateMsg() const override;
    const mavros_msgs::PositionTarget* getSetpointPtr() override;
    void handleManual(ControlFSM &fsm) override;
};

#endif
