#ifndef SHUTDOWN_STATE_HPP
#define SHUTDOWN_STATE_HPP
#include "StateInterface.hpp"

///Disables drone
class ShutdownState : public StateInterface {
public:
    ShutdownState();
    void handleEvent(ControlFSM& fsm, const EventData& event) override;
    void loopState(ControlFSM& fsm) override;
    void stateBegin(ControlFSM& fsm, const EventData& event) override;
    std::string getStateName() const override { return "Shutdown"; }
    const mavros_msgs::PositionTarget* getSetpoint() override;
};

#endif