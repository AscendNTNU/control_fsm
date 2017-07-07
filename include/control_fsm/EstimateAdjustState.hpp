#ifndef ESTIMATE_ADJUST_STATE_HPP
#define ESTIMATE_ADJUST_STATE_HPP
#include "StateInterface.hpp"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

///Adjusting position estimate
class EstimateAdjustState : public StateInterface {
private:
    EventData _cmd;
    ros::Subscriber _perceptionPosSub;
    geometry_msgs::PoseStamped _perPose;

    struct {
        ros::Duration delayTime;
        ros::Time startTime;
        bool enabled;
    } _delayedTransition;
public:
    EstimateAdjustState();
    void handleEvent(ControlFSM& fsm, const EventData& event) override;
    void stateBegin(ControlFSM& fsm, const EventData& event) override;
    void loopState(ControlFSM& fsm); //Uncomment if needed
    std::string getStateName() const override { return "EstimatorAdjust"; }

    void stateInit(ControlFSM &fsm) override;

    const mavros_msgs::PositionTarget* getSetpoint();
    void handleManual(ControlFSM &fsm) override;
    void perceptionPosCB(const geometry_msgs::PoseStamped& pose);
};
#endif
