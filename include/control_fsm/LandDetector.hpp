//
// Created by haavard on 01.06.17.
//

#ifndef CONTROL_FSM_LANDDETECTOR_HPP
#define CONTROL_FSM_LANDDETECTOR_HPP

#include <ros/ros.h>
#include <ascend_msgs/BoolStamped.h>

class ControlFSM;

class LandDetector {

private:
	std::string _topic;
	ControlFSM* _pFsm = nullptr;
	ros::NodeHandle _nh;
	ros::Subscriber _sub;
	ascend_msgs::BoolStamped _lastMsg;
	void landCB(const ascend_msgs::BoolStamped& msg);

public:
	LandDetector(std::string topic);
	LandDetector(std::string topic, ControlFSM* _pFsm);
	bool isOnGround();
	bool isReady();
};

#endif //CONTROL_FSM_LANDDETECTOR_HPP
