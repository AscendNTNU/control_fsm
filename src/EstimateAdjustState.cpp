#include "control_fsm/EstimateAdjustState.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include <control_fsm/FSMConfig.hpp>
#include "control_fsm/ControlFSM.hpp"

EstimateAdjustState::EstimateAdjustState() {
    _setpoint.type_mask = default_mask | IGNORE_PX | IGNORE_PY; //State should transition before this is needed, but just in case
}

void EstimateAdjustState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidCMD()) {
        if(_cmd.isValidCMD()) {
            event.eventError("ABORT old CMD first!");
            fsm.handleFSMWarn("ABORT old CMD before sending new!");
        } else { 
            _cmd = event;
        }
    } else if(event.eventType == EventType::REQUEST) {
        if(event.request == RequestType::ABORT && _cmd.isValidCMD()) {
            _cmd = EventData();
            _cmd.eventError("ABORT request!");
            fsm.handleFSMDebug("ABORTING command, but estimateadjust cant be aborted!");
        } else {
            fsm.handleFSMWarn("Illegal transition request");
        }
    } else {
        fsm.handleFSMDebug("Ignoring event");
    }
}

void EstimateAdjustState::loopState(ControlFSM& fsm) {
    auto& pos = fsm.getPositionXYZ()->pose.position;
    double dx = _perPose.pose.position.x - pos.x;
    double dy = _perPose.pose.position.y - pos.y;

    if(std::pow(dx, 2) + std::pow(dy, 2) < std::pow(FSMConfig::EstimatesIsEqualMargin, 2)) {
        if(!_delayedTransition.enabled) {
            _delayedTransition.enabled = true;
            _delayedTransition.startTime = ros::Time::now();
        }
        if(ros::Time::now() - _delayedTransition.startTime > _delayedTransition.delayTime) {
            if(_cmd.isValidCMD()) {
                fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, _cmd);
                _cmd = EventData();
            } else {
                RequestEvent event(RequestType::BLINDHOVER);
                fsm.transitionTo(ControlFSM::BLINDHOVERSTATE, this, event);
            }
        }

    } else {
        _delayedTransition.enabled = false;
    }
}

void EstimateAdjustState::stateBegin(ControlFSM& fsm, const EventData& event) {
    fsm.handleFSMError("EstimateAdjust has not been properly implemented!! Take manual control!");
    _setpoint.yaw = (float) fsm.getMavrosCorrectedYaw();

}

const mavros_msgs::PositionTarget* EstimateAdjustState::getSetpoint() {
    _setpoint.header.stamp = ros::Time::now();
    return &_setpoint;
}

void EstimateAdjustState::handleManual(ControlFSM &fsm) {
    fsm.handleFSMWarn("Lost OFFBOARD while adjusting position estimates! Do NOT switch back to OFFBOARD. Can lead to undefined behaviour!");
    //TODO Should it transition to MANUALFLIGHTSTATE?
}

void EstimateAdjustState::stateInit(ControlFSM &fsm) {
    _perceptionPosSub = fsm._nodeHandler.subscribe(FSMConfig::PerceptionPosTopic, 1, &EstimateAdjustState::perceptionPosCB, this);
}

void EstimateAdjustState::perceptionPosCB(const geometry_msgs::PoseStamped& pose) {
    _perPose = pose;
}

