#ifndef GO_TO_STATE_HPP
#define GO_TO_STATE_HPP
#include "state_interface.hpp"
#include <ros/ros.h>
#include <ascend_msgs/PointArrayStamped.h>
#include <memory>

namespace go_to_impl {
    enum class LocalState{INIT_PLAN, GOTO, REACHED_POINT, ABORT, COMPLETED};
}
///Moves drone to XYZ
class GoToState : public StateInterface {
private:
    //Local state interface
    friend go_to_impl::LocalState initPlanStateHandler(GoToState& s);
    friend go_to_impl::LocalState goToStateHandler(GoToState& s);
    friend go_to_impl::LocalState pointReachedStateHandler(GoToState& s);
    friend go_to_impl::LocalState completedStateHandler(GoToState& s);
    friend go_to_impl::LocalState abortStateHandler(GoToState& s);

    ///Get nodehandle, constructed on first use!
    static ros::NodeHandle& getNodeHandler();
    ///Path planner ROS service client
    ros::ServiceClient path_planner_client_;
    ///Path planner plan sub
    ros::Subscriber path_planner_sub_;
    ///When was plan requested
    ros::Time path_requested_stamp_;
    ///Last recieved plan
    ascend_msgs::PointArrayStamped::ConstPtr last_plan_;
    
    struct {
        ros::Time started;
        bool enabled = false;
        ros::Duration delayTime;
    } delay_transition_;

    ///Current command
    EventData cmd_;

    ///Is state active flag
    bool is_active_ = false;
    ///Plan callback 
    void planCB(ascend_msgs::PointArrayStamped::ConstPtr msg_p);

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
