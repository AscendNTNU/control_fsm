#ifndef ACTION_LIB_SERVER_HPP
#define ACTION_LIB_SERVER_HPP
#include <functional>
#include <actionlib/server/simple_action_server.h>
#include <control_fsm/GoToXYZAction.h>
#include <control_fsm/LandGBAction.h>
#include <LandXYAction.h>

template <typename ActionType>
class ActionLibServer {
private:
	ros::NodeHandle nh_;
	std::function<void()> onComplete_ = [](){};
	std::function<void()> onError_ = [](){};
	actionlib::SimpleActionServer<ActionType> as_
public:
	ActionLibServer(ros::NodeHandle nh, std::string name);
	void setOnCompleteCB(std::function<void()> cb) { onComplete_ = cb; }
	void setOnErrorCB(std::function<void()> cb) { onError_ = cb; }

};

template<typename ActionType>
ActionLibServer::ActionLibServer(ros::NodeHandle nh, std::string name) {
	nh_ = ros::NodeHandle(nh);
	as_ = actionlib::SimpleActionServer<ActionType>(nh_, name, false);
}





#endif