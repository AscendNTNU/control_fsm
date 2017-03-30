#ifndef TAKE_OFF_STATE_HPP
#define TAKE_OFF_STATE_HPP
#include "StateInterface.hpp"

class TakeoffState : public StateInterface {
private:
	double _takeoffaltitude = -1; //Invalid takeoff altitude as default
public:
	TakeoffState();
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	void stateBegin(ControlFSM& fsm, const EventData& event) override;
	void loopState(ControlFSM& fsm) override;
	std::string getStateName() const override { return "Takeoff";}
	const mavros_msgs::PositionTarget* getSetpoint() override;
};


#endif
