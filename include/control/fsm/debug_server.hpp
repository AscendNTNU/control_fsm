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

    bool handleDebugEvent(Request& req, Response& resp);
    EventData generateDebugEvent(Request&);
    //Remove copyconstructor and operator=
    DebugServer operator=(const DebugServer&) = delete;
    DebugServer(const DebugServer&) = delete;

public:
    DebugServer();
    std::queue<EventData> getAndClearQueue();
    bool isQueueEmpty() { return event_queue_.empty(); }


};

#endif
