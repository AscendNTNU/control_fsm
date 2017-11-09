#ifndef DEBUG_SERVER_HPP
#define DEBUG_SERVER_HPP

#include "control_fsm.hpp"
#include <ascend_msgs/ControlFSMEvent.h>
#include <queue>



class DebugServer {
    using Response = ascend_msgs::ControlFSMEvent::Response;
    using Request = ascend_msgs::ControlFSMEvent::Request;
private:
    ros::ServiceServer server_;
    ros::NodeHandle nh_;
    ///Queue containing events
    std::queue<EventData> event_queue_;
    ///Service callback
    bool handleDebugEvent(Request& req, Response& resp);
    ///Generates message
    EventData generateDebugEvent(Request&);
    //Remove copyconstructor and operator=
    DebugServer operator=(const DebugServer&) = delete;
    DebugServer(const DebugServer&) = delete;

public:
    ///Constructor
    DebugServer();
    ///Moves queue 
    std::queue<EventData> getAndClearQueue();
    ///Checks if queue is empty
    bool isQueueEmpty() { return event_queue_.empty(); }


};

#endif
