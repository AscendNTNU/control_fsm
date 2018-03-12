#ifndef GO_TO_STATE_HPP
#define GO_TO_STATE_HPP
#include "state_interface.hpp"
#include <ros/ros.h>
#include <ascend_msgs/PointArrayStamped.h>
#include <memory>
#include <functional>

///Moves drone to XYZ
class GoToState : public StateInterface {
private:
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
    std::string getStateName() const override { return "GoTo";}
    ascend_msgs::ControlFSMState getStateMsg() const override;
    const mavros_msgs::PositionTarget* getSetpointPtr() override;
    bool stateIsReady(ControlFSM &fsm) override;
    void handleManual(ControlFSM &fsm) override;

    ///Handles delayed transition when position is reached
    void destinationReached(ControlFSM &fsm, bool z_reached);
};

//Only make these available for unit testing
#ifdef CONTROL_FSM_UNIT_TEST
#include <geometry_msgs/TwistStamped.h>
bool droneNotMoving(const geometry_msgs::TwistStamped& vel);
double calculatePathYaw(double dx, double dy);
#endif

#endif
