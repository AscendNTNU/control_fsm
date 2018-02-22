#include <ros/ros.h>

#include "control/fsm/control_fsm.hpp"
#include "control/fsm/action_server.hpp"
#include "control/tools/config.hpp"
#include <ascend_msgs/ControlFSMEvent.h>
#include <std_msgs/String.h>
#include "control/tools/obstacle_avoidance.hpp"
#include <control/tools/logger.hpp>
#include "control/fsm/debug_server.hpp"
#include <ascend_msgs/StringService.h>


//How often is setpoints published to flightcontroller?
constexpr float SETPOINT_PUB_RATE = 30.0f; //In Hz

constexpr char mavrosSetpointTopic[] = "mavros/setpoint_raw/local";

//Returns nodes namespace
bool nameServiceCB(ascend_msgs::StringService::Request &, ascend_msgs::StringService::Response &response) {
    response.data = ros::names::append(ros::this_node::getNamespace(), ros::this_node::getName());
    return true;
}

int main(int argc, char** argv) {
    using control::Config;
    //Init ros and nodehandles
    ros::init(argc, argv, "control_fsm_main");
    ros::NodeHandle n;

    //Load ros params
    Config::loadParams();

    if(!Config::require_all_data_streams || !Config::require_obstacle_detection) {
        control::handleWarnMsg("One or more debug param features is activated!");
    }

    //FSM
    ControlFSM fsm;
    
    using ascend_msgs::StringServiceRequest;
    using ascend_msgs::StringServiceResponse;
    using ascend_msgs::ControlFSMState;

    //Set up neccesary publishers
    ros::Publisher setpoint_pub = n.advertise<mavros_msgs::PositionTarget>(mavrosSetpointTopic, 1);
    ros::Publisher fsm_on_state_changed_pub = n.advertise<ControlFSMState>(Config::fsm_state_changed_topic, Config::fsm_status_buffer_size);

    ros::ServiceServer namespace_service = n.advertiseService("/control_fsm_node_name", nameServiceCB);

    //Set up debug server
    DebugServer debugServer;

    //Spin once to get first messages
    ros::spinOnce();

    //Set FSM callbacks
    fsm.setOnStateChangedCB([&](){
        auto msg = fsm.getState()->getStateMsg();
        fsm_on_state_changed_pub.publish(msg);
    });

    //Wait for all systems to initalize and position to become valid
    control::handleInfoMsg("Waiting for necessary data streams!");
    while(ros::ok() && !fsm.isReady()) {
        ros::Duration(2.0).sleep();
        ros::spinOnce();
    }
    control::handleInfoMsg("Necessary data streams are ready!");

    //Actionserver is started when the system is ready
    ActionServer action_server;

    //Preflight is finished and system is ready for use!
    /**************************************************/
    control::handleInfoMsg("FSM is ready!");
    fsm.startPreflight(); //Transition to preflight!
    //Used to maintain a fixed loop rate
    ros::Rate loopRate(SETPOINT_PUB_RATE);
    //Main loop
    while(ros::ok()) {

        //Handle all incoming messages - generates fsm events
        ros::spinOnce();

        //Run action server events
        action_server.run(&fsm);

        //Handle debugevents
        if(!debugServer.isQueueEmpty()) {
            auto event_queue = debugServer.getAndClearQueue();
            while(!event_queue.empty()) {
                fsm.handleEvent(event_queue.front());
                event_queue.pop();
            }
        }

        //Run current FSM state loop
        fsm.loopCurrentState();

        //Publish completed setpoint
        setpoint_pub.publish(fsm.getMavrosSetpoint());

        //Sleep for remaining time
        loopRate.sleep();
    }

    return 0;
}


