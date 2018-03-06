#include "control/fsm/go_to_state.hpp"
#include "control/fsm/control_fsm.hpp"
#include <control/tools/logger.hpp>
#include <control/exceptions/pose_not_valid_exception.hpp>
#include <control/fsm/go_to_state.hpp>
#include "control/tools/config.hpp"
#include "control/tools/target_tools.hpp"

constexpr double PI = 3.14159265359;
constexpr double MAVROS_YAW_CORRECTION_PI_HALF = 3.141592653589793 / 2.0;

GoToState::GoToState() : StateInterface::StateInterface() {
    setpoint_.type_mask = default_mask;
}

void GoToState::handleEvent(ControlFSM& fsm, const EventData& event) {
    if(event.isValidRequest()) {
        if(event.request == RequestType::ABORT) {
            if(cmd_.isValidCMD()) {
                cmd_.eventError("ABORT");
                cmd_ = EventData();
            }
            fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, event);
        } else if(event.request == RequestType::POSHOLD) {
            if(cmd_.isValidCMD()) {
                control::handleWarnMsg("ABORT CMD before sending manual request!");
            } else {
                fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, event);
            }
        } else if(event.request == RequestType::GOTO) {
            if(cmd_.isValidCMD()) {
                control::handleWarnMsg("ABORT CMD before sending manual request!");
            } else {
                fsm.transitionTo(ControlFSM::GO_TO_STATE, this, event);
            }
        } else {
            control::handleWarnMsg("Illegal transiton request");
        }
    } else if(event.isValidCMD()) {
        if(cmd_.isValidCMD()) {
            event.eventError("ABORT request should be sent before new command");
        } else {
            if(event.command_type == CommandType::TAKEOFF) {
                //Drone already in air!
                event.finishCMD();
            } else {
                //All other command needs to go via the GOTO state
                fsm.transitionTo(ControlFSM::GO_TO_STATE, this, event);
            }
        }
    }
}

void GoToState::stateBegin(ControlFSM& fsm, const EventData& event) {

    //TAKEOFF cmd should not be processed by FSM
    if(event.isValidCMD(CommandType::TAKEOFF)) {
        event.eventError("Goto begin cmd bug!!");
        RequestEvent abort_event(RequestType::ABORT);
        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
        control::handleErrorMsg("TAKEOFF CMD should not be processed by GoTo - BUG!");
        return;
    }

    cmd_ = event;
    //Has not arrived yet
    delay_transition_.enabled = false;

    //Is target altitude too low?
    bool target_alt_too_low = cmd_.position_goal.z < control::Config::min_in_air_alt;
    if(target_alt_too_low) {
        control::handleWarnMsg("Target altitude is too low");
    }
    if(!event.position_goal.xyz_valid || target_alt_too_low) {
        if(cmd_.isValidCMD()) {
            event.eventError("No valid position target");
            cmd_ = EventData();
        }
        RequestEvent abort_event(RequestType::ABORT);
        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
        return;
    }

    auto tf = control::DroneHandler::getGlobal2LocalTf();

    // Set setpoint in local frame
    setpoint_.position.x = cmd_.position_goal.x + tf.transform.translation.x;
    setpoint_.position.y = cmd_.position_goal.y + tf.transform.translation.y;
    setpoint_.position.z = cmd_.position_goal.z + tf.transform.translation.z;
    try {
        ///Calculate yaw setpoint
        using control::pose::quat2yaw;
        using control::getMavrosCorrectedTargetYaw;
        using control::DroneHandler;
        auto quat = DroneHandler::getCurrentLocalPose().pose.orientation;
        setpoint_.yaw = static_cast<float>(getMavrosCorrectedTargetYaw(quat2yaw(quat)));
    } catch(const std::exception& e) {
        //Critical bug - no recovery
        //Transition to position hold if no pose available
        control::handleCriticalMsg(e.what());
        RequestEvent abort_event(RequestType::ABORT);
        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
    }
}

void GoToState::stateEnd(ControlFSM& fsm, const EventData& event) {
}

void GoToState::loopState(ControlFSM& fsm) {
    try {
        //Check that position data is valid
        if(!control::DroneHandler::isGlobalPoseValid()) {
            throw control::PoseNotValidException();
        }

        using control::pose::quat2mavrosyaw;
        //Get pose
        auto global_pose = control::DroneHandler::getCurrentGlobalPose();
        //Get reference to position in pose
        auto& global_position = global_pose.pose.position;
        //Get reference to orientation in pose
        auto& quat = global_pose.pose.orientation;
        //Calculate distance to target
        double delta_x = global_position.x - cmd_.position_goal.x;
        double delta_y = global_position.y - cmd_.position_goal.y;
        double delta_z = global_position.z - cmd_.position_goal.z;
        //Check if we're close enough
        using std::pow;
        using std::fabs;
        using control::Config;
        bool xy_reached = (pow(delta_x, 2) + pow(delta_y, 2)) <= pow(Config::dest_reached_margin, 2);
        bool z_reached = (fabs(delta_z) <= Config::altitude_reached_margin);
        bool yaw_reached = (fabs(quat2mavrosyaw(quat) - setpoint_.yaw) <= Config::yaw_reached_margin);
        //If destination is reached, begin transition to another state
        if(xy_reached && z_reached && yaw_reached) {
            destinationReached(fsm);
        } else {
            delay_transition_.enabled = false;
        }
    } catch(const std::exception& e) {
        //Exceptions should never occur!
        control::handleCriticalMsg(e.what());
        //Go to PosHold
        if(cmd_.isValidCMD()) {
            cmd_.eventError("No position");
            cmd_ = EventData();
        }
        RequestEvent abort_event(RequestType::ABORT);
        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
    }
}

//Returns valid setpoint
const mavros_msgs::PositionTarget* GoToState::getSetpointPtr() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

//Initialize state
void GoToState::stateInit(ControlFSM& fsm) {
    using control::Config;
    //TODO Uneccesary variables - Config can be used directly
    //Set state variables
    delay_transition_.delayTime = ros::Duration(Config::go_to_hold_dest_time);

    control::handleInfoMsg("GoTo init completed!");
}

/**
 * @brief Returns a yaw that is a multiple of 90 degrees
 * @details Drone should fly as straight forward as possible
 * , but yaw should be a multiple of 90 degrees.
 * This method assumes dx and dy != 0 at the same time
 * @param dx difference in x
 * @param dy difference in y
 * @return Yaw angle in radians - not mavros corrected
 */
double calculatePathYaw(double dx, double dy) {
    //Avoid fatal error if dx and dy is too small
    //If method is used correctly this should NEVER be a problem
    if(std::fabs(dx * dx + dy * dy) < 0.001) {
        return 0;
    }
    /*
    angle = acos(([dx, dy] dot [1,0]) / (norm([dx, dy]) * norm([1,0]))) = acos(dx / (norm([dx, dy]) * 1))
    */
    double angle = std::acos(dx / std::sqrt(dx * dx + dy * dy));

    //Select closest multiple of 90 degrees
    if(angle > 3 * PI / 4) {
        angle = PI;
    } else if(angle > PI / 4) {
        angle = PI / 2.0;
    } else {
        angle = 0;
    }
    //Invert if dy is negative
    if (dy < 0) {
        angle = -angle;
    }

    return angle;
}

bool GoToState::stateIsReady(ControlFSM& fsm) {
    return true;
}

void GoToState::handleManual(ControlFSM& fsm) {
    cmd_.eventError("Lost OFFBOARD");
    cmd_ = EventData();
    RequestEvent manual_event(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUAL_FLIGHT_STATE, this, manual_event);
}

//Check if velocity is close enough to zero
bool droneNotMoving(const geometry_msgs::TwistStamped& target) {
    using control::Config;
    using std::pow;
    auto& t_l = target.twist.linear;
    //Calculate square velocity
    double dx_sq = pow(t_l.x, 2);
    double dy_sq = pow(t_l.y, 2);
    double dz_sq = pow(t_l.z, 2);
    return (dx_sq + dy_sq + dz_sq) < pow(Config::velocity_reached_margin, 2);
}

void GoToState::destinationReached(ControlFSM& fsm) {
    //Transition to correct state
    if(cmd_.isValidCMD()) {
        switch(cmd_.command_type) {
            case CommandType::LANDXY: {
                //If no valid twist data it's unsafe to land
                if(!control::DroneHandler::isTwistValid()) {
                    control::handleErrorMsg("No valid twist data, unsafe to land! Transitioning to poshold");
                    cmd_.eventError("Unsafe to land!");
                    cmd_ = EventData();
                    RequestEvent abort_event(RequestType::ABORT);
                    fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
                    return;
                }
                //Check if drone is moving
                if(droneNotMoving(control::DroneHandler::getCurrentTwist())) {
                    //Hold current position for a duration - avoiding unwanted velocity before doing anything else
                    if(!delay_transition_.enabled) {
                        delay_transition_.started = ros::Time::now();
                        delay_transition_.enabled = true;

                        if(cmd_.isValidCMD()) {
                            cmd_.sendFeedback("Destination reached, letting drone slow down before transitioning!");
                        }
                    }
                    //Delay transition
                    if(ros::Time::now() - delay_transition_.started < delay_transition_.delayTime) {
                        return;
                    }
                    //If all checks passed - land!
                    fsm.transitionTo(ControlFSM::LAND_STATE, this, cmd_);
                } else {
                    //If drone is moving, reset delayed transition
                    delay_transition_.enabled = false;
                }
                break;
            }
                //NOTE: Land groundrobot algorithm not implemented yet, so this is commented out
                /*
                case CommandType::LANDGB:
                    fsm.transitionTo(ControlFSM::TRACK_GB_STATE, this, cmd_);
                    break;
                */
            case CommandType::GOTOXYZ: {
                cmd_.finishCMD();
                RequestEvent done_event(RequestType::POSHOLD);
                //Attempt to hold position target
                done_event.position_goal = cmd_.position_goal;
                fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, done_event);
            }
                break;
            default:
                control::handleWarnMsg("Unrecognized command type");
                break;
        }
    } else {
        RequestEvent pos_hold_event(RequestType::POSHOLD);
        pos_hold_event.position_goal = cmd_.position_goal;
        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, pos_hold_event);
    }

    delay_transition_.enabled = false;
}



ascend_msgs::ControlFSMState GoToState::getStateMsg() {
    using ascend_msgs::ControlFSMState;
    ControlFSMState msg;
    msg.name = getStateName();
    msg.state_id = ControlFSMState::GO_TO_STATE;
    return msg;
}

