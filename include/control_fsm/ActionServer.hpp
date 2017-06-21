#ifndef ACTION_SERVER_HPP
#define ACTION_SERVER_HPP
#include <actionlib/server/simple_action_server.h>
#include <control_fsm/ControlFSM.hpp>
#include <ascend_msgs/ControlFSMAction.h>
#include <ros/ros.h>

class ActionServer {
private:

    ros::NodeHandle nh_;
    ///Is an action already running?
    bool actionIsRunning_ = false;
    ///Pointer to fsm - used to send events directly to fsm
    ControlFSM* pFsm_ = nullptr;
    ///Actionserver
    actionlib::SimpleActionServer<ascend_msgs::ControlFSMAction> as_;
    ///Callback for when new action is recieved
    void goalCB();
    ///Callback for when to preempt action
    void preemptCB();
    ///Start goto action - send goto cmd to FSM
    void startGoTo(const ascend_msgs::ControlFSMGoal& goal);
    ///Start landxy action - send landxy cmd to FSM
    void startLandXY(const ascend_msgs::ControlFSMGoal& goal);
    ///Start landgb - NB not in use!!
    void startLandGB(const ascend_msgs::ControlFSMGoal& goal);
public:
    ActionServer(ControlFSM* pFsm);
};

#endif