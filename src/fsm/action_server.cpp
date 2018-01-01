#include <control/fsm/action_server.hpp>
#include <control/fsm/event_data.hpp>
#include <control/fsm/control_fsm.hpp>
#include <iostream>
#include <control/tools/logger.hpp>
#include <control/tools/config.hpp>

ActionServer::ActionServer(ControlFSM* fsm_p) : fsm_p_(fsm_p), as_(nh_, "controlNodeActionServer", false) {
    if(!ros::isInitialized()) {
        throw control::ROSNotInitializedException();
    }
    as_.registerGoalCallback(boost::bind(&ActionServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&ActionServer::preemptCB, this));
    as_.start();
}

void ActionServer::goalCB() {
    //Check if an action is already running
    if(action_is_running_) {
        RequestEvent abort_event(RequestType::ABORT);
        fsm_p_->handleEvent(abort_event);
    }

    auto goal = *(as_.acceptNewGoal());
    using GOALTYPE = ascend_msgs::ControlFSMActionGoal::_goal_type;
    switch(goal.cmd) {
        case GOALTYPE::GO_TO_XYZ:
            startGoTo(goal);
            break;
        case GOALTYPE::LAND_AT_POINT:
            startLandXY(goal);
            break;
        case GOALTYPE::LAND_ON_TOP_OF:
            startLandGB(goal);
            break;
        case GOALTYPE::SEARCH:
            control::handleWarnMsg("SEARCH not yet implemented");
            break;
        default:
            control::handleErrorMsg("[Control Action Server] Not a valid or available action");
            as_.setPreempted();
            break;
    }
}
//Preempt action and send abort to fsm
void ActionServer::preemptCB() {
    if(action_is_running_) {
        RequestEvent abort_event(RequestType::ABORT);
        fsm_p_->handleEvent(abort_event);
    }
    as_.setPreempted();
}

//If goal is goto, send valid goto cmd to fsm
void ActionServer::startGoTo(const ascend_msgs::ControlFSMGoal& goal) {
    GoToXYZCMDEvent go_to_event(goal.x, goal.y, goal.y);
    go_to_event.setOnCompleteCallback([this]() {
        ascend_msgs::ControlFSMResult result;
        result.finished = true;
        action_is_running_ = false;
        as_.setSucceeded(result);
    });

    go_to_event.setOnFeedbackCallback([this](std::string msg) {
        ascend_msgs::ControlFSMFeedback fb;
        fb.progression = msg;
        as_.publishFeedback(fb);
    });

    go_to_event.setOnErrorCallback([this](std::string msg) {
        ROS_WARN("[Control ActionServer] CMD error: %s", msg.c_str());
        ascend_msgs::ControlFSMResult result;
        result.finished = false;
        action_is_running_ = false;
        as_.setAborted(result);
    });
    fsm_p_->handleEvent(go_to_event);
}
//If goal is landxy, send valid landxy cmd to fsm
void ActionServer::startLandXY(const ascend_msgs::ControlFSMGoal& goal) {
    LandXYCMDEvent land_xy_event(goal.x, goal.y);
    land_xy_event.setOnCompleteCallback([this]() {
        ascend_msgs::ControlFSMResult result;
        result.finished = true;
        action_is_running_ = false;
        as_.setSucceeded(result);
    });

    land_xy_event.setOnFeedbackCallback([this](std::string msg) {
        ascend_msgs::ControlFSMFeedback fb;
        fb.progression = msg;
        as_.publishFeedback(fb);
    });

    land_xy_event.setOnErrorCallback([this](std::string msg) {
        control::handleWarnMsg(std::string("CMD error: ") + msg);
        ascend_msgs::ControlFSMResult result;
        result.finished = false;
        action_is_running_ = false;
        as_.setAborted(result);
    });
    fsm_p_->handleEvent(land_xy_event);
}

void ActionServer::startLandGB(const ascend_msgs::ControlFSMGoal& goal) {
    //TODO Implement when landgb procedure is decided
    ROS_WARN("[Control ActionServer] LandGB not implemented!!");
    ascend_msgs::ControlFSMResult result;
    result.finished = false;
    action_is_running_ = false;
    as_.setAborted(result);
}

void ActionServer::startSearch(const ascend_msgs::ControlFSMGoal& goal) {
    GoToXYZCMDEvent search_event(goal.x, goal.y, control::Config::gb_search_altitude);
    search_event.setOnCompleteCallback([this](){
        ascend_msgs::ControlFSMResult result;
        result.finished = true;
        action_is_running_ = false;
        as_.setSucceeded(result);
    });
    search_event.setOnFeedbackCallback([this](std::string msg) {
        ascend_msgs::ControlFSMFeedback fb;
        fb.progression = msg;
        as_.publishFeedback(fb);
    });
    search_event.setOnErrorCallback([this](std::string msg) {
        control::handleWarnMsg(std::string("CMD error: ") + msg);
        ascend_msgs::ControlFSMResult result;
        result.finished = false;
        action_is_running_ = false;
        as_.setAborted(result);
    });
    fsm_p_->handleEvent(search_event);


}
