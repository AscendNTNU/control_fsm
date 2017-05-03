#ifndef GO_TO_STATE_HPP
#define GO_TO_STATE_HPP
#include "StateInterface.hpp"
#include <ros/ros.h>
#include <ascend_msgs/PathPlannerPlan.h>
#include <memory>
#include <functional>

#define DEFAULT_DEST_REACHED_MARGIN 0.3
#define DEFAULT_SETPOINT_REACHED_MARGIN 0.3
#define DEFAULT_DEST_REACHED_DELAY 0.5
#define DEFAULT_YAW_REACHED_MARGIN 0.02

///Moves drone to XYZ 
class GoToState : public StateInterface {
private:

	struct {
		bool completed = false;
		std::function<void()> publish = [](){};
	} _safePublisher;

	struct {
		ros::Time started;
		bool enabled = false;
		ros::Duration delayTime;
	} _delayTransition;

	EventData _cmd;
	///Publisher for the current position - will start the path planner
	ros::Publisher _posPub;
	///Publisher for the desired target
	ros::Publisher _targetPub;
	///Publisher for obstacles
	ros::Publisher _obsPub;
	///Subscriber for path plan
	ros::Subscriber _planSub;
	///Is state active flag
	bool _isActive = false;
	///Dynamic allocated nodehandle
	std::unique_ptr<ros::NodeHandle> _pnh;

	///Contains the latest flight path recieved
	struct {
		ascend_msgs::PathPlannerPlan plan;
		bool valid = false;
		int index = 0;
	} _currentPlan;

	///Margin used to determine if we have arrived at our destination or not
	float _destReachedMargin = DEFAULT_DEST_REACHED_MARGIN;
	///Margin used to determine if we are close enough to a setpoint to switch
	float _setpointReachedMargin = DEFAULT_SETPOINT_REACHED_MARGIN;
	///Margin used to determine if we are close enough to target yaw
	float _yawReachedMargin = DEFAULT_YAW_REACHED_MARGIN;
	///Callback for path planner
	void pathRecievedCB(const ascend_msgs::PathPlannerPlan::ConstPtr& msg);
	/**
	 * @brief Returns a yaw that is a multiple of 90 degrees 
	 * @details Drone should fly as straight forward as possible
	 * , but yaw should be a multiple of 90 degrees.
	 * This method assumes dx and dy != 0 at the same time
	 * @param dx difference in x
	 * @param dy difference in y
	 * 
	 * @return Yaw angle in radians - not mavros corrected
	 */
	double calculatePathYaw(double dx, double dy);
public:
	GoToState();
	void stateInit(ControlFSM& fsm);
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	void stateBegin(ControlFSM& fsm, const EventData& event) override;
	void loopState(ControlFSM& fsm) override;
	void stateEnd(ControlFSM& fsm, const EventData& event) override;
	std::string getStateName() const { return "GoTo";}
	const mavros_msgs::PositionTarget* getSetpoint();
};

#endif