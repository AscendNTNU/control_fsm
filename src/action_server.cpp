#include <control_fsm/action_server.hpp>
#include <control_fsm/event_data.hpp>
#include <control_fsm/control_fsm.hpp>
#include <iostream>
ActionServer::ActionServer(ControlFSM* pFsm) : as_(nh_, "controlNodeActionServer", false) {
    this->p_fsm_ = pFsm;
    as_.registerGoalCallback(boost::bind(&ActionServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&ActionServer::preemptCB, this));
    as_.start();
}

void ActionServer::goalCB() {
    //Check if an action is already running
    if(actionIsRunning_) {
        RequestEvent abortEvent(RequestType::ABORT);
        p_fsm_->handleEvent(abortEvent);
        if(actionIsRunning_) {
            ROS_WARN("[Control Action Server] CMD not stopping after abort - bug!");
        }
    }

    auto goal = *(as_.acceptNewGoal());
    typedef ascend_msgs::ControlFSMActionGoal::_goal_type GOALTYPE;
    switch(goal.goalType) {
        case GOALTYPE::GOTOXYZ:
            startGoTo(goal);
            break;
        case GOALTYPE::LANDXY:
            startLandXY(goal);
            break;
        /* case GOALTYPE::LANDGB:
            startLandGB(goal);
            break; */
        default:
            ROS_ERROR("[Control Action Server] Not a valid or available action");
            as_.setPreempted();
            break;
    }
}
//Preempt action and send abort to fsm
void ActionServer::preemptCB() {
    if(actionIsRunning_) {
        RequestEvent abortEvent(RequestType::ABORT);
        p_fsm_->handleEvent(abortEvent);
        if(actionIsRunning_) {
            ROS_WARN("[Control Action Server] CMD not properly terminated on abort - bug!");
        }
    }
    as_.setPreempted();
}

//If goal is goto, send valid goto cmd to fsm
void ActionServer::startGoTo(const ascend_msgs::ControlFSMGoal& goal) {
    GoToXYZCMDEvent goToEvent(goal.x, goal.y, goal.y);
    goToEvent.setOnCompleteCallback([this]() {
        ascend_msgs::ControlFSMResult result;
        result.finished = true;
        actionIsRunning_ = false;
        as_.setSucceeded(result);
    });

    goToEvent.setOnFeedbackCallback([this](std::string msg) {
        ascend_msgs::ControlFSMFeedback fb;
        fb.progression = msg;
        as_.publishFeedback(fb);
    });

    goToEvent.setOnErrorCallback([this](std::string msg) {
        ROS_WARN("[Control ActionServer] CMD error: %s", msg.c_str());
        ascend_msgs::ControlFSMResult result;
        result.finished = false;
        actionIsRunning_ = false;
        as_.setAborted(result);
    });
    p_fsm_->handleEvent(goToEvent);
}
//If goal is landxy, send valid landxy cmd to fsm
void ActionServer::startLandXY(const ascend_msgs::ControlFSMGoal& goal) {
    LandXYCMDEvent landXYEvent(goal.x, goal.y);
    landXYEvent.setOnCompleteCallback([this]() {
        ascend_msgs::ControlFSMResult result;
        result.finished = true;
        actionIsRunning_ = false;
        as_.setSucceeded(result);
    });

    landXYEvent.setOnFeedbackCallback([this](std::string msg) {
        ascend_msgs::ControlFSMFeedback fb;
        fb.progression = msg;
        as_.publishFeedback(fb);
    });

    landXYEvent.setOnErrorCallback([this](std::string msg) {
        ROS_WARN("[Control ActionServer] CMD error: %s", msg.c_str());
        ascend_msgs::ControlFSMResult result;
        result.finished = false;
        actionIsRunning_ = false;
        as_.setAborted(result);
    });
    p_fsm_->handleEvent(landXYEvent);
}

void ActionServer::startLandGB(const ascend_msgs::ControlFSMGoal& goal) {
    //TODO Implement when landgb procedure is decided
    ROS_WARN("[Control ActionServer] LandGB not implemented!!");
    ascend_msgs::ControlFSMResult result;
    result.finished = false;
    actionIsRunning_ = false;
    as_.setAborted(result);
}

