#ifndef FSM_CONFIG_HPP
#define FSM_CONFIG_HPP
#include <iostream>
#include <ros/ros.h>

class FSMConfig {
public:
	///Are we close enough to the target?
	static double DestReachedMargin;
	///Default hover altitude in case of blind hover
	static double BlindHoverAlt;
	///Takeoff altitude
	static double TakeoffAltitude;
	///Are we close enough to target altitude
	static double AltitudeReachedMargin;
	///Are we close enough to setpoint
	static double SetpointReachedMargin;
	///Is yaw close enough?
	static double YawReachedMargin;
	///Topic to recieve path plan
	static std::string PathPlannerPlanTopic;
	///Topic to send drone position
	static std::string PathPlannerPosTopic;
	///Topic to send path target
	static std::string PathPlannerTargetTopic;
	///Topic to send obstacles to planner
	static std::string PathPlannerObsTopic;
	///Topic to publish FSM error msg
	static std::string FSMErrorTopic;
	///Topic to publish FSM warn msg
	static std::string FSMWarnTopic;
	///Topic to publish FSM info msg
	static std::string FSMInfoTopic;
	///Topic to publish FSM state changed
	static std::string FSMStateChangedTopic;
	///Buffer size used for FSMError, FSMWarn etc
	static int FSMStatusBufferSize;
	///Time gotostate waits before transitioning
	static double GoToHoldDestTime;
	///Distance used to determine if goto yaw should be calculated
	static double NoYawCorrectDist;
	///Altitude where the drone is safe from all obstacles
	static double SafeHoverAltitude;
	///Drone safezone
	static double ObstacleTooCloseDist;
	///Topic to listen for info about obstacles
	static std::string LidarTopic;
	///Load paramaters
	static void loadParams();
	///Finished drone will require all datastreams to be available
	static bool RequireAllDataStreams;

};

#endif