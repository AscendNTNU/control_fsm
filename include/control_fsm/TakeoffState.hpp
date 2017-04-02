#ifndef TAKE_OFF_STATE_HPP
#define TAKE_OFF_STATE_HPP
#include "StateInterface.hpp"

#define TAKEOFF_ALTITUDE 1.0f 
#define TAKEOFF_ALTITUDE_REACHED_THRESHOLD 0.1

class TakeoffState : public StateInterface {
private:
	EventData _cmd;
public:
	TakeoffState();
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	void stateBegin(ControlFSM& fsm, const EventData& event) override;
	void loopState(ControlFSM& fsm) override;
	std::string getStateName() const override { return "Takeoff";}
	const mavros_msgs::PositionTarget* getSetpoint() override;
};


#endif
