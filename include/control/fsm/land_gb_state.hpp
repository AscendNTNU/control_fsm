#ifndef INTERACT_GB_STATE_HPP
#define INTERACT_GB_STATE_HPP
#include "state_interface.hpp"

///Interacts with a ground robot
class LandGBState : public StateInterface {
private:
    EventData cmd_;
public:
    LandGBState();
    void handleEvent(ControlFSM& fsm, const EventData& event) override;
    void stateBegin(ControlFSM& fsm, const EventData& event) override;
    void loopState(ControlFSM& fsm) override;
    std::string getStateName() const override { return "LandGB"; }
    const mavros_msgs::PositionTarget* getSetpointPtr() override;
    void handleManual(ControlFSM &fsm) override;
};

#endif
