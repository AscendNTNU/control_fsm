#include "control_fsm/shutdown_state.hpp"
#include "control_fsm/control_fsm.hpp"

ShutdownState::ShutdownState() {
    setpoint_.type_mask = default_mask | SETPOINT_TYPE_IDLE;
}

void ShutdownState::handleEvent(ControlFSM& fsm, const EventData& event) {
    fsm.handleFSMWarn("Shutdown state, all events are ignored!");
}

void ShutdownState::loopState(ControlFSM& fsm) {
    //Not used
}

void ShutdownState::stateBegin(ControlFSM& fsm, const EventData& event) {
}

//Make sure to return setpoint_ (make sure it will stay in memory!)
const mavros_msgs::PositionTarget* ShutdownState::getSetpoint() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

void ShutdownState::handleManual(ControlFSM &fsm) {
    //Do nothing
}
