#include "control_fsm/ShutdownState.hpp"
#include "control_fsm/ControlFSM.hpp"

ShutdownState::ShutdownState() {
    _setpoint.type_mask = default_mask | SETPOINT_TYPE_IDLE;
}

void ShutdownState::handleEvent(ControlFSM& fsm, const EventData& event) {
    fsm.handleFSMWarn("Shutdown state, all events are ignored!");
}

void ShutdownState::loopState(ControlFSM& fsm) {
    //TODO Perform neccesary shutdown procedures
}

void ShutdownState::stateBegin(ControlFSM& fsm, const EventData& event) {
}

//Make sure to return _setpoint (make sure it will stay in memory!)
const mavros_msgs::PositionTarget* ShutdownState::getSetpoint() {
    _setpoint.header.stamp = ros::Time::now();
    return &_setpoint;
}

void ShutdownState::handleManual(ControlFSM &fsm) {
    //Do nothing
}
