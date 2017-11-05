#include <ros/ros.h>

#include "control/fsm/control_fsm.hpp"
#include "control/fsm/action_server.hpp"
#include "control/tools/config.hpp"
#include <ascend_msgs/ControlFSMEvent.h>
#include <std_msgs/String.h>
#include <control/tools/logger.hpp>
#include "control/fsm/debug_server.hpp"


//How often is setpoints published to flightcontroller?
constexpr float SETPOINT_PUB_RATE = 30.0f; //In Hz

constexpr char mavrosSetpointTopic[] = "mavros/setpoint_raw/local";

int main(int argc, char** argv) {
    using control::Config;
    //Init ros and nodehandles
    ros::init(argc, argv, "control_fsm_main");
    ros::NodeHandle n;
    ros::NodeHandle np("~");

    //Load ros params
    control::Config::loadParams();

    if(!Config::require_all_data_streams || !Config::require_obstacle_detection) {
        control::handleWarnMsg("One or more debug param features is activated!");
    }

    //Statemachine instance
    auto fsm_p = ControlFSM::getSharedInstancePtr();
    //Set up neccesary publishers
    ros::Publisher setpointPub = n.advertise<mavros_msgs::PositionTarget>(mavrosSetpointTopic, 1);
    ros::Publisher fsmOnStateChangedPub = n.advertise<std_msgs::String>(control::Config::fsm_state_changed_topic, Config::fsm_status_buffer_size);
    ros::Publisher fsmOnErrorPub = n.advertise<std_msgs::String>(Config::fsm_error_topic, Config::fsm_status_buffer_size);
    ros::Publisher fsmOnInfoPub = n.advertise<std_msgs::String>(Config::fsm_info_topic, Config::fsm_status_buffer_size);
    ros::Publisher fsmOnWarnPub = n.advertise<std_msgs::String>(Config::fsm_warn_topic, Config::fsm_status_buffer_size);

    //Set up debug server
    DebugServer debugServer(fsm_p.get());

    //Spin once to get first messages
    ros::spinOnce();

    //Set FSM callbacks
    fsm_p->setOnStateChangedCB([&](){
        std_msgs::String msg;
        msg.data = fsm_p->getState()->getStateName();
        fsmOnStateChangedPub.publish(msg);
    });


    //Wait for all systems to initalize and position to become valid
    control::handleInfoMsg("Waiting for necessary data streams!");
    while(ros::ok() && !fsm_p->isReady()) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    control::handleInfoMsg("Necessary data streams are ready!");

    //Actionserver is started when the system is ready
    ActionServer cmdServer(fsm_p.get());

    //Preflight is finished and system is ready for use!
    /**************************************************/
    control::handleInfoMsg("FSM is ready!");
    fsm_p->startPreflight(); //Transition to preflight!
    //Used to maintain a fixed loop rate
    ros::Rate loopRate(SETPOINT_PUB_RATE);
    //Main loop
    while(ros::ok()) {
        //Get latest messages
        ros::spinOnce(); //Handle all incoming messages - generates fsm events
        fsm_p->loopCurrentState(); //Run current FSM state loop

        //Publish setpoints at gived rate
        const mavros_msgs::PositionTarget* pSetpoint = fsm_p->getSetpoint();
        setpointPub.publish(*pSetpoint);

        //Sleep for remaining time
        loopRate.sleep();
    }

    return 0;
}


