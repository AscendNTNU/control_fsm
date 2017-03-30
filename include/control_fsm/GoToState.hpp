#ifndef GO_TO_STATE_HPP
#define GO_TO_STATE_HPP
#include "StateInterface.hpp"

class GoToState : public StateInterface {
private:
	EventData _lastEvent;
public:
	GoToState();
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	void stateBegin(ControlFSM& fsm, const EventData& event) override;
	void loopState(ControlFSM& fsm) override;
	std::string getStateName() const { return "GoTo";}
	const mavros_msgs::PositionTarget* getSetpoint();
};

#endif