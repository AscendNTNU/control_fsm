#ifndef TAKE_OFF_STATE_HPP
#define TAKE_OFF_STATE_HPP
#include "state_interface.hpp"
#include <ros/ros.h>

#define DEFAULT_TAKEOFF_ALTITUDE 1.0f 
#define DEFAULT_TAKEOFF_ALTITUDE_REACHED_MARGIN 0.1

///Takeoff state
class TakeoffState : public StateInterface {
private:
    event_data _cmd;
    double _altitude_reached_margin = DEFAULT_TAKEOFF_ALTITUDE_REACHED_MARGIN;
public:
    TakeoffState();
    void handleEvent(ControlFSM& fsm, const event_data& event) override;
    void stateBegin(ControlFSM& fsm, const event_data& event) override;
    void loopState(ControlFSM& fsm) override;
    std::string getStateName() const override { return "Takeoff";}
    const mavros_msgs::PositionTarget* getSetpoint() override;
    void handleManual(ControlFSM &fsm) override;
};


#endif
