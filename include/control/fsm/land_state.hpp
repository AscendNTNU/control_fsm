#ifndef LAND_STATE_HPP
#define LAND_STATE_HPP
#include "state_interface.hpp"

///Lands the drone
class LandState : public StateInterface {
private:
    EventData cmd_;
public:
    LandState();
    void handleEvent(ControlFSM& fsm, const EventData& event) override;
    void stateBegin(ControlFSM& fsm, const EventData& event) override;
    void loopState(ControlFSM& fsm) override;
    std::string getStateName() const override { return "Land"; }
    ascend_msgs::ControlFSMState getStateMsg() override;
    const mavros_msgs::PositionTarget* getSetpointPtr() override;
    void handleManual(ControlFSM &fsm) override;
};
#endif
