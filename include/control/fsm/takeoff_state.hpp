#ifndef TAKE_OFF_STATE_HPP
#define TAKE_OFF_STATE_HPP
#include "state_interface.hpp"
#include <ros/ros.h>

#define DEFAULT_TAKEOFF_ALTITUDE 1.0f 
#define DEFAULT_TAKEOFF_ALTITUDE_REACHED_MARGIN 0.1

///Forward decleration
class ControlFSM;

///Takeoff state
class TakeoffState : public StateInterface {
private:
    EventData cmd_;
public:
    TakeoffState();
    void handleEvent(ControlFSM& fsm, const EventData& event) override;
    void stateBegin(ControlFSM& fsm, const EventData& event) override;
    void loopState(ControlFSM& fsm) override;
    ascend_msgs::ControlFSMState getStateMsg() override;
    std::string getStateName() const override { return "Takeoff";}
    const mavros_msgs::PositionTarget* getSetpointPtr() override;
    void handleManual(ControlFSM &fsm) override;
};

#endif
