
#include "control_fsm/debug_server.hpp"
#include "control_fsm/event_data.hpp"

DebugServer::DebugServer(ControlFSM* pFsm) : pFsm_(pFsm) {
    //Must be constructed after ros init
    assert(ros::isInitialized());
    //Can't be nullptr
    assert(pFsm_);
    //Advertise service! 
    server_ = nh_.advertiseService("control_fsm_debug", &DebugServer::handleDebugEvent, this);
}

bool DebugServer::handleDebugEvent(Request& req, Response& resp) {
    if(!pFsm_->isReady()) {
        pFsm_->handleFSMWarn("Preflight not complete, however FSM do respond to debug requests! Be careful!!");
    }
    EventData event = generateDebugEvent(req);
    //If request event is not valid
    if(event.eventType == EventType::REQUEST && !event.isValidRequest()) {
        resp.accepted = false;
        resp.errorMsg = "Not valid request event";
        return true;
    } 
    //If command event is not valid
    if(event.eventType == EventType::COMMAND && !event.isValidCMD()) {
        resp.accepted = false;
        resp.errorMsg = "Not valid cmd event";
        return true;
    }
    //If there is no valid event
    if(event.eventType == EventType::NONE) {
        resp.accepted = false;
        resp.errorMsg = "No valid event";
        return true;
    }

    if(event.isValidCMD()) {
        event.setOnCompleteCallback([](){
            ROS_INFO("[Control FSM Debug] Manual CMD finished");
        });
        event.setOnFeedbackCallback([](std::string msg){
            ROS_INFO("[Control FSM Debug] Manual CMD feedback: %s", msg.c_str());
        });
        event.setOnErrorCallback([](std::string errMsg) {
            ROS_WARN("[Control FSM Debug] Manual CMD error: %s", errMsg.c_str());
        });
    }
    pFsm_->handleEvent(event);
    resp.accepted = true;
    resp.stateName = pFsm_->getState()->getStateName();
    return true;

}

EventData DebugServer::generateDebugEvent(ascend_msgs::ControlFSMEvent::Request&req) {
    EventData event;
    //Lambda expression returning correct eventtype
    event.eventType = ([&]() -> EventType{
        using REQ = ascend_msgs::ControlFSMEvent::Request;
        switch(req.eventType) {
            case REQ::REQUEST: return EventType::REQUEST;
            case REQ::COMMAND: return EventType::COMMAND;
            case REQ::AUTONOMOUS: return EventType::AUTONOMOUS;
            case REQ::MANUAL: return EventType::MANUAL;
            case REQ::GROUNDDETECTED: return EventType::GROUNDDETECTED;
            default: return EventType::NONE;
        }
    })();

    if(event.eventType == EventType::REQUEST) {
        //Lambda expression returning correct requesttype
        event.request = ([&]() -> RequestType {
            using REQ = ascend_msgs::ControlFSMEvent::Request;
            switch(req.requestType) {
                case REQ::ABORT: return RequestType::ABORT;
                case REQ::BEGIN: return RequestType::BEGIN;
                case REQ::END: return RequestType::END;
                case REQ::PREFLIGHT: return RequestType::PREFLIGHT;
                case REQ::IDLE: return RequestType::IDLE;
                case REQ::SHUTDOWN: return RequestType::SHUTDOWN;
                case REQ::TAKEOFF: return RequestType::TAKEOFF;
                case REQ::BLINDHOVER: return RequestType::BLINDHOVER;
                case REQ::POSHOLD: return RequestType::POSHOLD;
                case REQ::GOTO: return RequestType::GOTO;
                case REQ::LAND: return RequestType::LAND;
                case REQ::BLINDLAND: return RequestType::BLINDLAND;
                //case REQ::TRACKGB: return RequestType::TRACKGB;
                //case REQ::INTERGB: return RequestType::INTERGB;
                case REQ::ESTIMATORADJ: return RequestType::ESTIMATORADJ;
                case REQ::MANUALFLIGHT: return RequestType::MANUALFLIGHT;
                default: return RequestType::NONE;
            }
        })();
        if(event.request == RequestType::GOTO) {
            event.positionGoal = PositionGoalXYZ(req.x, req.y, req.z);
        }
    } else if(event.eventType == EventType::COMMAND) {
        //Lambda expression returning correct commandEvent
        event = ([&]() -> EventData{
            using REQ = ascend_msgs::ControlFSMEvent::Request;
            switch(req.commandType) {
                case REQ::LANDXY: return LandXYCMDEvent(req.x, req.y);
                case REQ::GOTOXYZ: return GoToXYZCMDEvent(req.x, req.y, req.z);
                    //case REQ::LANDGB: return LandGBCMDEvent();
                default:
                    EventData e;
                    e.eventType = EventType::COMMAND;
                    e.commandType = CommandType::NONE;
                    return e;
            }
        })();
    }
    return event;
}

