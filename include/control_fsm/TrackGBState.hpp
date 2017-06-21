#ifndef TRACK_GB_STATE_HPP
#define TRACK_GB_STATE_HPP
#include "StateInterface.hpp"

///Tracks ground robot
class TrackGBState : public StateInterface {
public:
    TrackGBState();
    void handleEvent(ControlFSM& fsm, const EventData& event) override;
    void stateBegin(ControlFSM& fsm, const EventData& event) override;
    void loopState(ControlFSM& fsm) override;
    std::string getStateName() const override { return "TrackGroundRobot"; }
    const mavros_msgs::PositionTarget* getSetpoint() override;
};


#endif