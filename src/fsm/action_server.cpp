#include <control/fsm/action_server.hpp>
#include <control/tools/logger.hpp>
#include <control/tools/config.hpp>

constexpr int MAX_ITERATIONS = 100;

ActionServer::ActionServer() : as_(nh_, "control_fsm_action_server", false) {
    if(!ros::isInitialized()) {
        throw control::ROSNotInitializedException();
    }
    action_state_service_ = nh_.advertiseService("control_fsm_action_state", &ActionServer::actionStateServiceCB, this);
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

    if(!state_s_.debug_enabled && !state_s_.ai_enabled) {
        control::handleWarnMsg("Action server not active");
        as_.setAborted();
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

    if(current_goal->caller_id != current_goal->CALLER_AI
       && current_goal->caller_id != current_goal->CALLER_DEBUGGER) {
        as_.setAborted();
        control::handleErrorMsg("Invalid action caller");
        return;
    }

    if(current_goal->caller_id == current_goal->CALLER_AI && !state_s_.ai_enabled) {
        as_.setAborted();
        control::handleWarnMsg("Action server not enabled for AI");
        return;
    }

    if(current_goal->caller_id == current_goal->CALLER_DEBUGGER && !state_s_.debug_enabled) {
        as_.setAborted();
        control::handleWarnMsg("Action server not enabled for DEBUGGER");
        return;
    }

    //Select find correct command
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
        });
    }
}

//If goal is goto, send valid goto cmd to fsm
void ActionServer::startGoTo(GoalSharedPtr goal_p, ControlFSM* fsm_p) {

    if(goal_p->z < control::Config::min_in_air_alt) {
        control::handleWarnMsg("Action goal target altitude is too low, aborting!");
        as_.setAborted();
        return;
    }

    GoToXYZCMDEvent go_to_event(goal_p->x, goal_p->y, goal_p->z);
    //Set callback to run on completion
    go_to_event.setOnCompleteCallback([this]() {
        onActionComplete();
    });
    //Set callback to run on feedback
    go_to_event.setOnFeedbackCallback([this](const std::string& msg) {
        onActionFeedback(msg);
    });
    //Set callback to run on error
    go_to_event.setOnErrorCallback([this](const std::string& msg) {
        onActionError(msg);
    });
    fsm_p->handleEvent(go_to_event);
    action_is_running_ = true;
}
//If goal is landxy, send valid landxy cmd to fsm
void ActionServer::startLandXY(GoalSharedPtr goal_p, ControlFSM* fsm_p) {
    LandXYCMDEvent land_xy_event(goal_p->x, goal_p->y);
    //Set callback to run on complete
    land_xy_event.setOnCompleteCallback([this]() {
        onActionComplete();
    });
    //Set callback to run on complete
    land_xy_event.setOnFeedbackCallback([this](const std::string& msg) {
        onActionFeedback(msg);
    });
    land_xy_event.setOnErrorCallback([this](const std::string& msg) {
        onActionError(msg);
    });
    fsm_p->handleEvent(land_xy_event);
    action_is_running_ = true;
}

void ActionServer::startLandGB(GoalSharedPtr goal_p, ControlFSM* fsm_p) {
    //TODO Implement when landgb procedure is decided
    ROS_WARN("[Control ActionServer] LandGB not implemented!!");
    ascend_msgs::ControlFSMResult result;
    action_is_running_ = false;
    as_.setAborted(result);
}

void ActionServer::startSearch(GoalSharedPtr goal_p, ControlFSM* fsm_p) {
    GoToXYZCMDEvent search_event(goal_p->x, goal_p->y, control::Config::gb_search_altitude);
    //Set callback to run on complete
    search_event.setOnCompleteCallback([this](){
        onActionComplete();
    });
    //Set callback to run on feedback
    search_event.setOnFeedbackCallback([this](const std::string& msg) {
        onActionFeedback(msg);
    });
    //Set callback to run on error
    search_event.setOnErrorCallback([this](const std::string& msg) {
        onActionError(msg);
    });
    //Run event in fsm
    fsm_p->handleEvent(search_event);
    action_is_running_ = true;

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

void ActionServer::onActionComplete() {
        ascend_msgs::ControlFSMResult result;
        action_is_running_ = false;
        as_.setSucceeded(result);
}

void ActionServer::onActionFeedback(const std::string& msg) {
        ascend_msgs::ControlFSMFeedback fb;
        fb.progression = msg;
        as_.publishFeedback(fb);
}

void ActionServer::onActionError(const std::string& msg) {
        control::handleWarnMsg(std::string("CMD error: ") + msg);
        ascend_msgs::ControlFSMResult result;
        action_is_running_ = false;
        as_.setAborted(result);
}

bool ActionServer::actionStateServiceCB(ActionServer::StateRequest& req, ActionServer::StateResponse& resp) {


    state_s_.ai_enabled = req.ai_enabled;
    state_s_.debug_enabled = req.debug_enabled;

    control::handleInfoMsg(std::string("AI Actions: ") + (state_s_.ai_enabled ? "Enabled" : "Disabled"));
    control::handleInfoMsg(std::string("DEBUGGER Actions: ") + (state_s_.debug_enabled ? "Enabled" : "Disabled"));
    resp.result = static_cast<unsigned char>(true);
    return true;
}
