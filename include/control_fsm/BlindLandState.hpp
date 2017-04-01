#ifndef BLIND_LAND_STATE_HPP
#define BLIND_LAND_STATE_HPP
#include "StateInterface.hpp"

class BlindLandState : public StateInterface {
public:
	BlindLandState();
	void handleEvent(ControlFSM& fsm, const EventData& event) override;
	void stateBegin(ControlFSM& fsm, const EventData& event) override;
	void loopState(ControlFSM& fsm) override;
	std::string getStateName() const { return "BlindLand";}
	const mavros_msgs::PositionTarget* getSetpoint();
};
#endif