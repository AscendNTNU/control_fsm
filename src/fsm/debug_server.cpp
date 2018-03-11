
#include <control/tools/logger.hpp>
#include "control/fsm/debug_server.hpp"
#include "control/fsm/event_data.hpp"

DebugServer::DebugServer() {
    //Make sure ROS is initialized
    if(!ros::isInitialized()) {
        throw control::ROSNotInitializedException();
    }
    //Advertise service! 
    using control::Config;
    server_ = nh_.advertiseService(Config::debug_server_topic, &DebugServer::handleDebugEvent, this);
}

bool DebugServer::handleDebugEvent(Request& req, Response& resp) {
    EventData event = generateDebugEvent(req);
    //If request event is not valid
    if(event.event_type == EventType::REQUEST && !event.isValidRequest()) {
        resp.accepted = false;
        resp.errorMsg = "Not valid request event";
        return true;
    } 
    //If command event is not valid
    if(event.event_type == EventType::COMMAND && !event.isValidCMD()) {
        resp.accepted = false;
        resp.errorMsg = "Not valid cmd event";
        return true;
    }
    //If there is no valid event
    if(event.event_type == EventType::NONE) {
        resp.accepted = false;
        resp.errorMsg = "No valid event";
        return true;
    }

    if(event.isValidCMD()) {
        event.setOnCompleteCallback([](){
                control::handleInfoMsg("Manual CMD finished");
        });
        event.setOnFeedbackCallback([](std::string msg){
                std::string to_print = "Manual CMD feedback: ";
                to_print += msg;
                control::handleInfoMsg(to_print);
        });
        event.setOnErrorCallback([](std::string err_msg) {
                std::string to_print = "Manual CMD error: ";
                to_print += err_msg;
                control::handleInfoMsg(to_print);
        });
    }
    event_queue_.push(event);
    resp.accepted = true;
    return true;

}

EventData DebugServer::generateDebugEvent(ascend_msgs::ControlFSMEvent::Request&req) {
    EventData event;
    //Lambda expression returning correct eventtype
    event.event_type = ([&]() -> EventType{
        using REQ = ascend_msgs::ControlFSMEvent::Request;
        switch(req.eventType) {
            case REQ::REQUEST: return EventType::REQUEST;
            case REQ::COMMAND: return EventType::COMMAND;
            case REQ::AUTONOMOUS: return EventType::AUTONOMOUS;
            case REQ::MANUAL: return EventType::MANUAL;
            default: return EventType::NONE;
        }
    })();

    if(event.event_type == EventType::REQUEST) {
        //Lambda expression returning correct requesttype
        event.request = ([&]() -> RequestType {
            using REQ = ascend_msgs::ControlFSMEvent::Request;
            switch(req.requestType) {
                case REQ::ABORT: return RequestType::ABORT;
                case REQ::BEGIN: return RequestType::BEGIN;
                case REQ::END: return RequestType::END;
                case REQ::PREFLIGHT: return RequestType::PREFLIGHT;
                case REQ::IDLE: return RequestType::IDLE;
                case REQ::TAKEOFF: return RequestType::TAKEOFF;
                case REQ::BLINDHOVER: return RequestType::BLINDHOVER;
                case REQ::POSHOLD: return RequestType::POSHOLD;
                case REQ::GOTO: return RequestType::GOTO;
                case REQ::LAND: return RequestType::LAND;
                //case REQ::TRACKGB: return RequestType::TRACKGB;
                //case REQ::INTERGB: return RequestType::INTERGB;
                case REQ::MANUALFLIGHT: return RequestType::MANUALFLIGHT;
                default: return RequestType::NONE;
            }
        })();
        if(event.request == RequestType::GOTO) {
            event.position_goal = PositionGoal(req.x, req.y, req.z);
        }
    } else if(event.event_type == EventType::COMMAND) {
        //Lambda expression returning correct commandEvent
        event = ([&]() -> EventData{
            using REQ = ascend_msgs::ControlFSMEvent::Request;
            switch(req.commandType) {
                case REQ::LANDXY: return LandXYCMDEvent(req.x, req.y);
                case REQ::GOTOXYZ: return GoToXYZCMDEvent(req.x, req.y, req.z);
                    //case REQ::LANDGB: return LandGBCMDEvent();
                case REQ::TAKEOFFCMD: return TakeoffCMDEvent();
                default:
                    EventData e;
                    e.event_type = EventType::COMMAND;
                    e.command_type = CommandType::NONE;
                    return e;
            }
        })();
    }
    return event;
}

std::queue<EventData> DebugServer::getAndClearQueue() {
    std::queue<EventData> temp;
    std::swap(temp, event_queue_);
    return temp; //Returned by move constructor
}

