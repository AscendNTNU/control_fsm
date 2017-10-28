#include <ros/ros.h>

#include "control_fsm/control_fsm.hpp"
#include "control_fsm/action_server.hpp"
#include "control_fsm/fsm_config.hpp"
#include <ascend_msgs/ControlFSMEvent.h>
#include <std_msgs/String.h>
#include "control_fsm/debug_server.hpp"


//How often is setpoints published to flightcontroller?
constexpr float SETPOINT_PUB_RATE = 30.0f; //In Hz

constexpr char mavrosSetpointTopic[] = "mavros/setpoint_raw/local";

int main(int argc, char** argv) {
    //Init ros and nodehandles
    ros::init(argc, argv, "control_fsm_main");
    ros::NodeHandle n;
    ros::NodeHandle np("~");

    //Load ros params
    FSMConfig::loadParams();

    if(!FSMConfig::require_all_data_streams || !FSMConfig::require_obstacle_detection) {
        ROS_WARN("One or more debug param features is activated!");
    }

    //Statemachine instance
    ControlFSM fsm;

    //Set up neccesary publishers
    ros::Publisher setpointPub = n.advertise<mavros_msgs::PositionTarget>(mavrosSetpointTopic, 1);
    ros::Publisher fsmOnStateChangedPub = n.advertise<std_msgs::String>(FSMConfig::fsm_state_changed_topic, FSMConfig::fsm_status_buffer_size);
    ros::Publisher fsmOnErrorPub = n.advertise<std_msgs::String>(FSMConfig::fsm_error_topic, FSMConfig::fsm_status_buffer_size);
    ros::Publisher fsmOnInfoPub = n.advertise<std_msgs::String>(FSMConfig::fsm_info_topic, FSMConfig::fsm_status_buffer_size);
    ros::Publisher fsmOnWarnPub = n.advertise<std_msgs::String>(FSMConfig::fsm_warn_topic, FSMConfig::fsm_status_buffer_size);

    //Set up debug server
    DebugServer debugServer(&fsm);

    //Spin once to get first messages
    ros::spinOnce();

    //Set FSM callbacks
    fsm.setOnStateChangedCB([&](){
        std_msgs::String msg;
        msg.data = fsm.getState()->getStateName();
        fsmOnStateChangedPub.publish(msg);
    });

    fsm.setOnFSMErrorCB([&](const std::string& errMsg) {
        std_msgs::String msg;
        msg.data = errMsg;
        fsmOnErrorPub.publish(msg);
    });

    fsm.setOnFSMWarnCB([&](const std::string& warnMsg) {
        std_msgs::String msg;
        msg.data = warnMsg;
        fsmOnWarnPub.publish(msg);
    });

    fsm.setOnFSMInfoCB([&](const std::string& infoMsg) {
        std_msgs::String msg;
        msg.data = infoMsg;
        fsmOnInfoPub.publish(msg);
    });


    //Wait for all systems to initalize and position to become valid
    fsm.handleFSMInfo("Waiting for necessary data streams!");
    while(ros::ok() && !fsm.isReady()) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    fsm.handleFSMInfo("Necessary data streams are ready!");

    //Actionserver is started when the system is ready
    ActionServer cmdServer(&fsm);

    //Preflight is finished and system is ready for use!
    /**************************************************/
    fsm.handleFSMInfo("FSM is ready!");
    fsm.startPreflight(); //Transition to preflight!
    //Used to maintain a fixed loop rate
    ros::Rate loopRate(SETPOINT_PUB_RATE);
    //Main loop
    while(ros::ok()) {
        //Get latest messages
        ros::spinOnce(); //Handle all incoming messages - generates fsm events
        fsm.loopCurrentState(); //Run current FSM state loop

        //Publish setpoints at gived rate
        const mavros_msgs::PositionTarget* pSetpoint = fsm.getSetpoint();
        setpointPub.publish(*pSetpoint);

        //Sleep for remaining time
        loopRate.sleep();
    }

    return 0;
}


