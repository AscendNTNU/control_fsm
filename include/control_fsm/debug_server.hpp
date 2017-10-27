#ifndef DEBUG_SERVER_HPP
#define DEBUG_SERVER_HPP

#include "control_fsm.hpp"
#include <ascend_msgs/ControlFSMEvent.h>



class DebugServer {
    using Response = ascend_msgs::ControlFSMEvent::Response;
    using Request = ascend_msgs::ControlFSMEvent::Request;
private:
    ControlFSM* fsm_p_ = nullptr;
    ros::ServiceServer server_;
    ros::NodeHandle nh_;

    bool handleDebugEvent(Request& req, Response& resp);
    EventData generateDebugEvent(Request&);
    //Remove copyconstructor and operator=
    DebugServer operator=(const DebugServer&) = delete;
    DebugServer(const DebugServer&) = delete;

public:
    DebugServer(ControlFSM* p_fsm);

};

#endif