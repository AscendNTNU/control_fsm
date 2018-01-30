#ifndef GO_TO_STATE_HPP
#define GO_TO_STATE_HPP
#include "state_interface.hpp"
#include <ros/ros.h>
#include <ascend_msgs/PathPlannerPlan.h>
#include <memory>
#include <functional>

#define DEFAULT_DEST_REACHED_MARGIN 0.3
#define DEFAULT_SETPOINT_REACHED_MARGIN 0.3
#define DEFAULT_YAW_REACHED_MARGIN 0.02



///Moves drone to XYZ 
class GoToState : public StateInterface {
private:

    struct {
        ros::Time started;
        bool enabled = false;
        ros::Duration delayTime;
    } delay_transition_;

    EventData cmd_;
    ///Is state active flag
    bool is_active_ = false;

    ///Margin used to determine if we have arrived at our destination or not
    double dest_reached_margin_ = DEFAULT_DEST_REACHED_MARGIN;
    ///Margin used to determine if we are close enough to a setpoint to switch
    double setpoint_reached_margin_ = DEFAULT_SETPOINT_REACHED_MARGIN;
    ///Margin used to determine if we are close enough to target yaw
    double yaw_reached_margin_ = DEFAULT_YAW_REACHED_MARGIN;
public:
    GoToState();
    void stateInit(ControlFSM& fsm) override;
    void handleEvent(ControlFSM& fsm, const EventData& event) override;
    // Get target position
    void stateBegin(ControlFSM& fsm, const EventData& event) override;
    // Poll drone position and set setpoint
    void loopState(ControlFSM& fsm) override;
    // 
    void stateEnd(ControlFSM& fsm, const EventData& event) override;
    std::string getStateName() const { return "GoTo";}
    const mavros_msgs::PositionTarget* getSetpointPtr();
    bool stateIsReady(ControlFSM &fsm) override;
    void handleManual(ControlFSM &fsm) override;

    ///Handles delayed transition when position is reached
    void destinationReached(ControlFSM &fsm);
};

//Only make these available for unit testing
#ifdef CONTROL_FSM_UNIT_TEST
#include <geometry_msgs/TwistStamped.h>
bool droneNotMoving(const geometry_msgs::TwistStamped& vel);
double calculatePathYaw(double dx, double dy);
#endif

#endif
