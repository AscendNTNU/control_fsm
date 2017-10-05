#ifndef ESTIMATE_ADJUST_STATE_HPP
#define ESTIMATE_ADJUST_STATE_HPP
#include "state_interface.hpp"

///Adjusting position estimate
class EstimateAdjustState : public StateInterface {
private:
    EventData cmd_;

public:
    EstimateAdjustState();
    void handleEvent(ControlFSM& fsm, const EventData& event) override;
    void stateBegin(ControlFSM& fsm, const EventData& event) override;
    void loopState(ControlFSM& fsm); //Uncomment if needed
    std::string getStateName() const override { return "EstimatorAdjust"; }
    const mavros_msgs::PositionTarget* getSetpoint();
    void handleManual(ControlFSM &fsm) override;
};
#endif
