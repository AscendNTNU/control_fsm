#include "control/fsm/go_to_state.hpp"
#include "control/fsm/control_fsm.hpp"
#include <control/tools/logger.hpp>
#include <control/exceptions/pose_not_valid_exception.hpp>
#include <control/fsm/go_to_state.hpp>
#include <tf2/LinearMath/Transform.h>
#include "control/tools/config.hpp"
#include "control/tools/target_tools.hpp"
#include <ascend_msgs/PathPlanner.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <functional>
#include <array>

using namespace go_to_impl;
constexpr double PI = 3.14159265359;
constexpr double MAVROS_YAW_CORRECTION_PI_HALF = 3.141592653589793 / 2.0;
//Gurantees construction on first use!
ros::NodeHandle& GoToState::getNodeHandler() {
    static ros::NodeHandle n;
    return n;
}

//Forward declaration
bool targetWithinArena(const tf2::Vector3& target);
bool droneNotMovingXY(const geometry_msgs::TwistStamped& target);

LocalState initPlanStateHandler(GoToState& s);
LocalState goToStateHandler(GoToState& s);
LocalState pointReachedStateHandler(GoToState& s);
LocalState slowStateHandler(GoToState& s);
LocalState abortStateHandler(GoToState& s);
LocalState completedStateHandler(GoToState& s);

LocalState local_state = LocalState::INIT_PLAN;

std::array<std::function<decltype(initPlanStateHandler)>,
           static_cast<int>(LocalState::NUM_STATES)> state_array = { 
    initPlanStateHandler,
    goToStateHandler,
    pointReachedStateHandler,
    slowStateHandler,
    abortStateHandler,
    completedStateHandler
};

std::function<decltype(initPlanStateHandler)> stateFunction = initPlanStateHandler;

//Calls pathplanner service and request next step in plan
LocalState initPlanStateHandler(GoToState& s) {
    //Get transforms and apply local -> global tf to get global goal.
    auto tf = control::DroneHandler::getLocal2GlobalTf();
    tf2::Transform tf_matrix;
    tf2::convert(tf.transform, tf_matrix);
    s.local_target_ = s.cmd_.position_goal_local.getVec3();
    auto global_target = tf_matrix * s.local_target_;

    //Checks if the point is valid.
    bool target_alt_too_low = s.local_target_.z() < control::Config::min_in_air_alt;
    bool invalid_target = !s.cmd_.position_goal_local.xyz_valid ||
                          !targetWithinArena(global_target)     ||
                          target_alt_too_low;

    if(target_alt_too_low) {
        control::handleWarnMsg("Target altitude too low?");
    }
    if(invalid_target) {
        s.cmd_.eventError("Invalid target!");
        return LocalState::ABORT;
    }
    
    //Request a new plan from the pathplanner.
    using PathServiceRequest = ascend_msgs::PathPlanner;
    PathServiceRequest req;
    req.request.cmd = PathServiceRequest::Request::MAKE_PLAN;
    req.request.goal_x = static_cast<float>(global_target.x());
    req.request.goal_y = static_cast<float>(global_target.y());

    if(!s.path_planner_client_.call(req)) {
        control::handleErrorMsg("Could not request path plan");
        s.cmd_.eventError("Could not request plan!");
        return LocalState::ABORT;
    }
    s.path_requested_stamp_ = ros::Time::now();

    return LocalState::GOTO;
}

//Sends setpoints and awaits arrival at point
LocalState goToStateHandler(GoToState& s) {
    bool last_plan_timeout = ros::Time::now() - s.last_plan_->header.stamp >
                             ros::Duration(control::Config::path_plan_timeout);
    bool plan_requested_timeout = ros::Time::now() - s.path_requested_stamp_ > 
                                  ros::Duration(control::Config::path_plan_timeout);
    
    //Checks if the plan has timed out or if the plan is empty
    if(last_plan_timeout || s.last_plan_->points.empty()) {
        if(plan_requested_timeout) {
            s.cmd_.eventError("Plan timeout or no points in plan!");
            return LocalState::ABORT;
        }
        return LocalState::GOTO;
    }
    
    auto& target_point = s.last_plan_->points[0];
    auto global_target = tf2::Vector3(target_point.x, target_point.y, s.cmd_.position_goal_local.z);
    auto tf = control::DroneHandler::getGlobal2LocalTf();
    tf2::Transform tf_matrix;
    tf2::convert(tf.transform, tf_matrix);
    s.local_target_ = tf_matrix * global_target; 
    s.setpoint_.position.x = s.local_target_.x();
    s.setpoint_.position.y = s.local_target_.y(); 
    s.setpoint_.position.z = s.local_target_.z();

    return LocalState::REACHED_POINT;
}

//Checks if the point is the last point of the plan
LocalState pointReachedStateHandler(GoToState& s) {
    auto tf = control::DroneHandler::getLocal2GlobalTf();
    tf2::Transform tf_matrix;
    tf2::convert(tf.transform, tf_matrix);
    auto global_target = tf_matrix * s.local_target_;
    if(control::Config::restrict_arena_boundaries) {
        if(!targetWithinArena(global_target)) {
            s.cmd_.eventError("Target outside arena!");
            return LocalState::ABORT;
        }
    }
    
    using control::pose::quat2mavrosyaw;
    auto local_pose = control::DroneHandler::getCurrentLocalPose();
    auto& local_position = local_pose.pose.position;
    auto& quat = local_pose.pose.orientation;
    
    double delta_x = local_position.x - s.cmd_.position_goal_local.x;
    double delta_y = local_position.y - s.cmd_.position_goal_local.y;
    double delta_z = local_position.z - s.cmd_.position_goal_local.z;
    double delta_xy_squared = pow(delta_x, 2) + pow(delta_y, 2);
    double delta_yaw = quat2mavrosyaw(quat) - s.setpoint_.yaw;

    using std::fabs;
    using control::Config;
    bool xy_reached = delta_xy_squared <= pow(Config::dest_reached_margin, 2);
    bool z_reached = delta_z <= fabs(Config::altitude_reached_margin);
    bool yaw_reached = (fabs(delta_yaw) <= Config::yaw_reached_margin);
    
    //This is messy, is there another more efficient way to do it?
    if (xy_reached && yaw_reached) {
        if(s.cmd_.command_type == CommandType::LANDXY) {
            //Altitude not important, but velocity is
            return LocalState::SLOW;
        } else if(z_reached) {
            //Altitude important before PosHold
            return LocalState::COMPLETED;
        }
    }
    return LocalState::GOTO;
}

LocalState slowStateHandler(GoToState& s) { 
    //TODO: Make the drone slow down. 
    if(droneNotMovingXY(control::DroneHandler::getCurrentTwist())) {
        return LocalState::COMPLETED;
    } else {
        return LocalState::SLOW;
    }
}

LocalState abortStateHandler(GoToState& s) {
    return LocalState::ABORT;
}

LocalState completedStateHandler(GoToState& s) {
    return LocalState::COMPLETED;
}

GoToState::GoToState() : StateInterface::StateInterface()  {
    using ascend_msgs::PointArrayStamped;
    last_plan_ = ascend_msgs::PointArrayStamped::ConstPtr(new ascend_msgs::PointArrayStamped);
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
            //All other command needs to go via the GOTO state
            fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, event);
        }
    }
}

bool targetWithinArena(const tf2::Vector3& target) {
        using control::Config;
        bool global_x_invalid = target.x() > Config::arena_highest_x ||
                                target.x() < Config::arena_lowest_x;
        bool global_y_invalid = target.y() > Config::arena_highest_y ||
                                target.y() < Config::arena_lowest_y;
        return !global_x_invalid && !global_y_invalid;
}

void GoToState::stateBegin(ControlFSM& fsm, const EventData& event) {

    cmd_ = event;
    local_state = LocalState::INIT_PLAN;
 
    // Set setpoint in local frame to target
    try {
        ///Calculate yaw setpoint
        using control::pose::quat2yaw;
        using control::getMavrosCorrectedTargetYaw;
        using control::DroneHandler;

        auto pose = DroneHandler::getCurrentLocalPose().pose;
        //Hold position while waiting for new plan!
        setpoint_.position.x = pose.position.x;
        setpoint_.position.y = pose.position.y;
        setpoint_.position.z = pose.position.z;
        setpoint_.yaw = static_cast<float>(getMavrosCorrectedTargetYaw(quat2yaw(pose.orientation)));
    } catch(const std::exception& e) {
        //Critical bug - no recovery
        //Transition to position hold if no pose available
        control::handleCriticalMsg(e.what());
        RequestEvent abort_event(RequestType::ABORT);
        fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
    }
}

void GoToState::stateEnd(ControlFSM& fsm, const EventData& event) {
    using PathServiceRequest = ascend_msgs::PathPlanner;
    PathServiceRequest req;
    req.request.cmd = PathServiceRequest::Request::ABORT;
    if(!path_planner_client_.call(req)) {
        control::handleErrorMsg("Failed to call path planner service");
    }
}

void GoToState::loopState(ControlFSM& fsm) {
    using control::Config;
    using control::DroneHandler;
    try {
        //Check that position data is valid
        if(!DroneHandler::isGlobalPoseValid()) {
            throw control::PoseNotValidException();
        }
        stateFunction = state_array[static_cast<int>(local_state)];
        local_state = stateFunction(*this);
        
        if(local_state == LocalState::ABORT) {
            RequestEvent abort_event(RequestType::ABORT);
            fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, abort_event);
        } else if(local_state == LocalState::COMPLETED) {
            //This should handle land and gotoxy differently
            if (cmd_.isValidCMD(CommandType::LANDXY)) {
                fsm.transitionTo(ControlFSM::LAND_STATE, this, cmd_);
            } else {
                cmd_.finishCMD();
                RequestEvent pos_hold_event(RequestType::POSHOLD);
                fsm.transitionTo(ControlFSM::POSITION_HOLD_STATE, this, pos_hold_event);
            }
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

void GoToState::planCB(ascend_msgs::PointArrayStamped::ConstPtr msg_p) {
    last_plan_ = msg_p;
}

//Initialize state
void GoToState::stateInit(ControlFSM& fsm) {
    using control::Config;
    using ascend_msgs::PointArrayStamped;
    using ascend_msgs::PathPlanner;
    auto& pc_topic = Config::path_planner_client_topic;
    auto& ps_topic = Config::path_planner_plan_topic;
    path_planner_client_ = getNodeHandler().serviceClient<PathPlanner>(pc_topic); 
    path_planner_sub_ = getNodeHandler().subscribe(ps_topic, 1, &GoToState::planCB, this);

    setStateIsReady();
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

void GoToState::handleManual(ControlFSM& fsm) {
    cmd_.eventError("Lost OFFBOARD");
    cmd_ = EventData();
    RequestEvent manual_event(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUAL_FLIGHT_STATE, this, manual_event);
}

//Check if velocity is close enough to zero
bool droneNotMovingXY(const geometry_msgs::TwistStamped& target) {
    using control::Config;
    using std::pow;
    auto& t_l = target.twist.linear;
    //Calculate square velocity
    double dx_sq = pow(t_l.x, 2);
    double dy_sq = pow(t_l.y, 2);
    return (dx_sq + dy_sq) < pow(Config::velocity_reached_margin, 2);
}

ascend_msgs::ControlFSMState GoToState::getStateMsg() const {
    using ascend_msgs::ControlFSMState;
    ControlFSMState msg;
    msg.name = getStateName();
    msg.state_id = ControlFSMState::GO_TO_STATE;
    return msg;
}

