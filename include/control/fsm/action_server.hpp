#ifndef ACTION_SERVER_HPP
#define ACTION_SERVER_HPP
#include <actionlib/server/simple_action_server.h>
#include <control/fsm/control_fsm.hpp>
#include <ascend_msgs/ControlFSMAction.h>
#include <ros/ros.h>
#include <queue>

class ActionServer {
private:
    ros::NodeHandle nh_;
    ///Is an action already running?
    bool action_is_running_ = false;
    ///Event queue
    std::queue<EventData> event_queue_;
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
    ///Constructor
    ActionServer();
    ///Request event queue
    std::queue<EventData>&& getAndClearQueue();
    ///Is queue empty
    bool isQueueEmpty() { return event_queue_.empty(); }
    
    
};

#endif
