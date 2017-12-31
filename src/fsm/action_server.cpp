#include <control/fsm/action_server.hpp>
#include <control/fsm/event_data.hpp>
#include <control/fsm/control_fsm.hpp>
#include <iostream>
ActionServer::ActionServer() : as_(nh_, "controlNodeActionServer", false) {
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
        event_queue_.push_back(abort_event);            
    }

    auto goal = *(as_.acceptNewGoal());
    using GOALTYPE = ascend_msgs::ControlFSMActionGoal::_goal_type;
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
            control::handleErrorMsg("[Control Action Server] Not a valid or available action");
            as_.setPreempted();
            break;
    }
}
//Preempt action and send abort to fsm
void ActionServer::preemptCB() {
    if(action_is_running_) {
        RequestEvent abort_event(RequestType::ABORT);
        event_queue_.push_back(abort_event);
    }
    as_.setPreempted();
}

//If goal is goto, send valid goto cmd to fsm
void ActionServer::startGoTo(const ascend_msgs::ControlFSMGoal& goal) {
    GoToXYZCMDEvent go_to_event(goal.x, goal.y, goal.y);
    go_to_event.setOnCompleteCallback([&as_, &action_is_running_]() {
        ascend_msgs::ControlFSMResult result;
        result.finished = true;
        action_is_running_ = false;
        as_.setSucceeded(result);
    });

    go_to_event.setOnFeedbackCallback([&as_](std::string msg) {
        ascend_msgs::ControlFSMFeedback fb;
        fb.progression = msg;
        as_.publishFeedback(fb);
    });

    go_to_event.setOnErrorCallback([&as_, &action_is_running_](std::string msg) {
        ROS_WARN("[Control ActionServer] CMD error: %s", msg.c_str());
        ascend_msgs::ControlFSMResult result;
        result.finished = false;
        action_is_running_ = false;
        as_.setAborted(result);
    });
    event_queue_.push_back(go_to_event);
}
//If goal is landxy, send valid landxy cmd to fsm
void ActionServer::startLandXY(const ascend_msgs::ControlFSMGoal& goal) {
    LandXYCMDEvent land_xy_event(goal.x, goal.y);
    land_xy_event.setOnCompleteCallback([&as, &action_is_running_]() {
        ascend_msgs::ControlFSMResult result;
        result.finished = true;
        action_is_running_ = false;
        as_.setSucceeded(result);
    });

    land_xy_event.setOnFeedbackCallback([&as_](std::string msg) {
        ascend_msgs::ControlFSMFeedback fb;
        fb.progression = msg;
        as_.publishFeedback(fb);
    });

    land_xy_event.setOnErrorCallback([&as_, &action_is_running_](std::string msg) {
        control::handleWarnMsg(std::string("CMD error: ") + msg);
        ascend_msgs::ControlFSMResult result;
        result.finished = false;
        action_is_running_ = false;
        as_.setAborted(result);
    });
    event_queue_.push_back(land_xy_event);
}

void ActionServer::startLandGB(const ascend_msgs::ControlFSMGoal& goal) {
    //TODO Implement when landgb procedure is decided
    ROS_WARN("[Control ActionServer] LandGB not implemented!!");
    ascend_msgs::ControlFSMResult result;
    result.finished = false;
    action_is_running_ = false;
    as_.setAborted(result);
}

std::queue<EventData>&& ActionServer::getAndClearQueue() {
    auto temp = std::move(event_queue_);
    event_queue_.clear();
    return temp;
}
