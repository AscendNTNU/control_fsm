//
// Created by haavard on 07.01.18.
//

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ascend_msgs/ControlFSMAction.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "action_interface_client");
    ros::NodeHandle n;
    actionlib::SimpleActionClient<ascend_msgs::ControlFSMAction> client("control_fsm_action_server", true);

    std::vector<std::string> commandline_options;
    commandline_options.reserve(static_cast<unsigned long>(argc - 1));
    for (int i = 1; i < argc; ++i) {
        auto toupper = [](std::string str) {
            for (char &it : str) {
                it = static_cast<char>(std::toupper(it));
            }
            return str;
        };
        commandline_options.emplace_back(toupper(argv[i]));
    }
    if (commandline_options.empty()) {
        ROS_ERROR("[FSM Action client] No enough arguments");
        return -1;
    }
    using GoalType = ascend_msgs::ControlFSMGoal;
    GoalType goal;
    auto &type_arg = commandline_options[0];
    if (type_arg == "GOTO") {
        if (commandline_options.size() != 4) {
            ROS_ERROR("[FSM Action client] No enough arguments");
            return -1;
        }
        try {
            float x_target = std::stof(commandline_options[1]);
            float y_target = std::stof(commandline_options[2]);
            float z_target = std::stof(commandline_options[3]);
            goal.x = x_target;
            goal.y = y_target;
            goal.z = z_target;
            goal.cmd = GoalType::GO_TO_XYZ;
        } catch (const std::invalid_argument &e) {
            ROS_ERROR("[FSM Action client] Parse error!");
            return -1;
        }
    } else if (type_arg == "LAND") {
        if (commandline_options.size() != 4) {
            ROS_ERROR("[FSM Action client] No enough arguments");
            return -1;
        }
        try {
            float x_target = std::stof(commandline_options[1]);
            float y_target = std::stof(commandline_options[2]);
            goal.x = x_target;
            goal.y = y_target;
            goal.cmd = GoalType::LAND_AT_POINT;
        } catch (const std::invalid_argument &e) {
            ROS_ERROR("[FSM Action client] Parse error!");
            return -1;
        }
    }

    ROS_INFO("[FSM Action client] Waiting for action server server");
    bool timeout = !client.waitForServer(ros::Duration(5.0));
    if (timeout) {
        ROS_ERROR("Server connection timed out!");
        return -1;
    }
    client.sendGoal(goal);
    timeout = !client.waitForResult(ros::Duration(2.0));
    if (timeout) {
        client.cancelGoal();
        ROS_ERROR("Action took to long!");
        return -1;
    }

    auto state = client.getState();
    if (state == state.SUCCEEDED) {
        ROS_INFO("Success!");
    } else {
        ROS_INFO("Failed!");
    }
}

