#include <control/fsm/action_server.hpp>
#include <control/tools/logger.hpp>
#include <control/tools/config.hpp>

constexpr int MAX_ITERATIONS = 100;

ActionServer::ActionServer() : as_(nh_, "controlNodeActionServer", false) {
    if(!ros::isInitialized()) {
        throw control::ROSNotInitializedException();
    }
    as_.registerGoalCallback(boost::bind(&ActionServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&ActionServer::preemptCB, this));
    as_.start();
}

void ActionServer::goalCB() {
    ///Delay execution
    event_queue.push([this](ControlFSM *fsm_p) {
        ActionServer::handleNewGoal(fsm_p);
    });
}

void ActionServer::handleNewGoal(ControlFSM *fsm_p) {
    //Check that there really is a new goal available
    if(!as_.isNewGoalAvailable()) {
        return;
    }

    //Get new goal
    current_goal = as_.acceptNewGoal();
    //Check that the client hasn't requested a preempt
    if(as_.isPreemptRequested()) {
        as_.setPreempted();
        return;
    }
    //Check if an action is already running
    if(action_is_running_) {
        RequestEvent abort_event(RequestType::ABORT);
        fsm_p->handleEvent(abort_event);
        if(action_is_running_) {
            control::handleErrorMsg("Action not terminated, error!");
        }
    }
    using GOALTYPE = ascend_msgs::ControlFSMActionGoal::_goal_type;
    switch(current_goal->cmd) {
        case GOALTYPE::GO_TO_XYZ:
            startGoTo(current_goal, fsm_p);
            break;
        case GOALTYPE::LAND_AT_POINT:
            startLandXY(current_goal, fsm_p);
            break;
        case GOALTYPE::LAND_ON_TOP_OF:
            startLandGB(current_goal, fsm_p);
            break;
        case GOALTYPE::SEARCH:
            startSearch(current_goal, fsm_p);
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
        event_queue.emplace([this](ControlFSM* fsm_p) {
            RequestEvent abort_event(RequestType::ABORT);
            fsm_p->handleEvent(abort_event);
            if(action_is_running_) {
                control::handleErrorMsg("Action not terminated, error!");
            }
            as_.setPreempted();
        });
    } else {
        as_.setPreempted();
    }
}

//If goal is goto, send valid goto cmd to fsm
void ActionServer::startGoTo(GoalSharedPtr goal_p, ControlFSM* fsm_p) {
    GoToXYZCMDEvent go_to_event(goal_p->x, goal_p->y, goal_p->y);
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
    fsm_p->handleEvent(go_to_event);
}
//If goal is landxy, send valid landxy cmd to fsm
void ActionServer::startLandXY(GoalSharedPtr goal_p, ControlFSM* fsm_p) {
    LandXYCMDEvent land_xy_event(goal_p->x, goal_p->y);
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
    fsm_p->handleEvent(land_xy_event);
}

void ActionServer::startLandGB(GoalSharedPtr goal_p, ControlFSM* fsm_p) {
    //TODO Implement when landgb procedure is decided
    ROS_WARN("[Control ActionServer] LandGB not implemented!!");
    ascend_msgs::ControlFSMResult result;
    result.finished = false;
    action_is_running_ = false;
    as_.setAborted(result);
}

void ActionServer::startSearch(GoalSharedPtr goal_p, ControlFSM* fsm_p) {
    GoToXYZCMDEvent search_event(goal_p->x, goal_p->y, control::Config::gb_search_altitude);
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
    fsm_p->handleEvent(search_event);


}

void ActionServer::run(ControlFSM *fsm_p) {
    if(event_queue.empty()) return;
    //Count number of events to run
    for(int i = 0; !event_queue.empty(); ++i) {
        if(i > MAX_ITERATIONS) {
            control::handleErrorMsg("Too many events in event queue");
            return;
        }
        //Run events in queue
        auto run_event = event_queue.front();
        run_event(fsm_p);
        event_queue.pop();
    }
}
