#ifndef INTERACT_GB_STATE_HPP
#define INTERACT_GB_STATE_HPP
#include "state_interface.hpp"

///Interacts with a ground robot
class InteractGBState : public StateInterface {
public:
    InteractGBState();
    void handleEvent(ControlFSM& fsm, const EventData& event) override;
    void stateBegin(ControlFSM& fsm, const EventData& event) override;
    void loopState(ControlFSM& fsm) override;
    std::string getStateName() const override { return "InteractWithGroundRobot"; }
    const mavros_msgs::PositionTarget* getSetpointPtr() override;
    ascend_msgs::ControlFSMState getStateMsg() const override;
    void handleManual(ControlFSM &fsm) override;
};

#endif
