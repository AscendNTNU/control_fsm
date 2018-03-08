#ifndef POSITION_HOLD_STATE_HPP
#define POSITION_HOLD_STATE_HPP
#include "state_interface.hpp"
#include <memory>
#include <ros/ros.h>
#include <ascend_msgs/PointArray.h>
#define DEFAULT_SAFE_HOVER_ALT 2.5

///Holds the current position
class PositionHoldState : public StateInterface {
    bool obstacle_avoidance_kicked_in_ = false;
public:
    PositionHoldState();
    void handleEvent(ControlFSM& fsm, const EventData& event) override;
    void stateBegin(ControlFSM& fsm, const EventData& event) override;
    void stateEnd(ControlFSM& fsm, const EventData& event) override;
    void stateInit(ControlFSM& fsm) override;
    std::string getStateName() const override { return "Position hold"; }
    ascend_msgs::ControlFSMState getStateMsg() override;
    const mavros_msgs::PositionTarget* getSetpointPtr() override;
    void handleManual(ControlFSM &fsm) override;
};

#endif
