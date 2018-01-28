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
    /**
     * @brief Returns a yaw that is a multiple of 90 degrees 
     * @details Drone should fly as straight forward as possible
     * , but yaw should be a multiple of 90 degrees.
     * This method assumes dx and dy != 0 at the same time
     * @param dx difference in x
     * @param dy difference in y
     * 
     * @return Yaw angle in radians - not mavros corrected
     */
    double calculatePathYaw(double dx, double dy);
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

#endif
