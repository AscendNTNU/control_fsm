#ifndef ACTION_SERVER_HPP
#define ACTION_SERVER_HPP
#include <actionlib/server/simple_action_server.h>
#include <control/fsm/control_fsm.hpp>
#include <ascend_msgs/ControlFSMAction.h>
#include <ascend_msgs/ControlFSMSetActionState.h>
#include <ros/ros.h>
#include <queue>

class ActionServer {
private:
    using FSMGoal = ascend_msgs::ControlFSMGoal::Type;
    using GoalSharedPtr = boost::shared_ptr<const FSMGoal>;
    using StateService = ascend_msgs::ControlFSMSetActionState;
    using StateRequest = StateService::Request;
    using StateResponse = StateService::Response;

    struct {
        bool ai_enabled = false;
        bool debug_enabled = false;
    } state_s_;

    ///Stores what the actionserver wants to do - since last run.
    std::queue<std::function<void(ControlFSM*)>> event_queue;
    ///Current goal
    GoalSharedPtr current_goal;
    ///ROS nodehandle
    ros::NodeHandle nh_;
    ///Is an action already running?
    bool action_is_running_ = false;
    ///Actionserver
    actionlib::SimpleActionServer<ascend_msgs::ControlFSMAction> as_;
    ///ROS Service to enable/disable action server
    ros::ServiceServer action_state_service_;

    bool actionStateServiceCB(StateRequest&, StateResponse&);
    ///Callback for when new action is recieved
    void goalCB();
    ///Callback for when to preempt action
    void preemptCB();
    ///Start correct action
    void handleNewGoal(ControlFSM* fsm_p);
    ///Start goto action - send goto cmd to FSM
    void startGoTo(GoalSharedPtr goal_p, ControlFSM* fsm_p);
    ///Start takeoff action - send takeoff cmd to FSM
    void startTakeoff(GoalSharedPtr goal_p, ControlFSM* fsm_p);
    ///Start landxy action - send landxy cmd to FSM
    void startLandXY(GoalSharedPtr goal_p, ControlFSM* fsm_p);
    ///Start landgb - NB not in use!!
    void startLandGB(GoalSharedPtr goal_p, ControlFSM* fsm_p);
    ///Start search - go to search alt
    void startSearch(GoalSharedPtr goal_p, ControlFSM* fsm_p);
    //Callback to run on action complete
    void onActionComplete();
    ///Callback to run on action feedback
    void onActionFeedback(const std::string& msg);
    ///Callback to run on action error
    void onActionError(const std::string& msg);
    
public:
    ///Constructor
    ActionServer();
    ///Run all generated events - uncoupled from the rest
    void run(ControlFSM* fsm_p);
};

#endif
