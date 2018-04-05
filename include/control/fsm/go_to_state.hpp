#ifndef GO_TO_STATE_HPP
#define GO_TO_STATE_HPP
#include "state_interface.hpp"
#include <ros/ros.h>
#include <ascend_msgs/PathPlannerPlan.h>
#include <memory>
#include <functional>

///Moves drone to XYZ
class GoToState : public StateInterface {
private:

    struct {
        ros::Time started;
        bool enabled = false;
        ros::Duration delayTime;
    } delay_transition_;

    ///Current command
    EventData cmd_;

    ///Local target to reach
    tf2::Vector3 local_target_;
public:
    GoToState();
    void stateInit(ControlFSM& fsm) override;
    void handleEvent(ControlFSM& fsm, const EventData& event) override;
    // Get target position
    void stateBegin(ControlFSM& fsm, const EventData& event) override;
    // Poll drone position and set setpoint
    void loopState(ControlFSM& fsm) override;
    void stateEnd(ControlFSM& fsm, const EventData& event) override;
    std::string getStateName() const override { return "GoTo";}
    ascend_msgs::ControlFSMState getStateMsg() const override;
    const mavros_msgs::PositionTarget* getSetpointPtr() override;
    void handleManual(ControlFSM &fsm) override;

    ///Handles transitions when position is reached
    void destinationReached(ControlFSM &fsm, bool z_reached);

    ///Handles landing transition
    void landingTransition(ControlFSM& fsm);
};

//Only make these available for unit testing
#ifdef CONTROL_FSM_UNIT_TEST
#include <geometry_msgs/TwistStamped.h>
bool droneNotMovingXY(const geometry_msgs::TwistStamped& vel);
double calculatePathYaw(double dx, double dy);
bool targetWithinArena(const tf2::Vector3& target);
#endif

#endif
