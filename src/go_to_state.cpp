#include "control_fsm/go_to_state.hpp"
#include "control_fsm/setpoint_msg_defines.h"
#include <ros/ros.h>
#include "control_fsm/control_fsm.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <geometry_msgs/Point32.h>
#include <ascend_msgs/PathPlannerPlan.h>
#include "control_fsm/fsm_config.hpp"

constexpr double PI = 3.14159265359;
constexpr double PI_HALF = 1.57079632679;


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
            fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event);
        } else if(event.request == RequestType::POSHOLD) {
            if(cmd_.isValidCMD()) {
                fsm.handleFSMWarn("ABORT CMD before sending manual request!");
            } else {
                fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event);
            }
        } else if(event.request == RequestType::GOTO) {
            if(cmd_.isValidCMD()) {
                fsm.handleFSMWarn("ABORT CMD before sending manual request!");
            } else {
                fsm.transitionTo(ControlFSM::GOTOSTATE, this, event);
            }
        } else {
            fsm.handleFSMWarn("Illegal transiton request");
        }
    } else if(event.isValidCMD()) {
        if(cmd_.isValidCMD()) {
            event.eventError("ABORT request should be sent before new command");
        } else {
            fsm.transitionTo(ControlFSM::GOTOSTATE, this, event); //Transition to itself
        }
    }
}

void GoToState::stateBegin(ControlFSM& fsm, const EventData& event) {
    isActive_ = true;
    cmd_ = event;
    //Current plan is invalid until new plan is recieved
    currentPlan_.valid = false;
    //Has not arrived yet
    delayTransition_.enabled = false;

    if(!event.positionGoal.valid) {
        if(cmd_.isValidCMD()) {
            event.eventError("No valid position target");
            cmd_ = EventData();
        }
        RequestEvent nEvent(RequestType::ABORT);
        fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, nEvent);
        return;
    }

    //Sets setpoint to current position - until planner is done
    const geometry_msgs::PoseStamped* pPose = fsm.getPositionXYZ();
    setpoint_.position.x = pPose->pose.position.x;
    setpoint_.position.y = pPose->pose.position.y;

    //Z setpoint can be set right away
    setpoint_.position.z = event.positionGoal.z;
    //Set yaw setpoint to desired target yaw
    setpoint_.yaw = (float) fsm.getMavrosCorrectedYaw();

    //Calculate the square distance from drone to target
    double deltaX = pPose->pose.position.x - event.positionGoal.x;
    double deltaY = pPose->pose.position.y - event.positionGoal.y;
    double posXYDistSquare = std::pow(deltaX, 2) + std::pow(deltaY, 2);

    //If only altitude is different, no need for pathplanner
    if(posXYDistSquare <= std::pow(destReachedMargin_, 2)) {
        if(cmd_.isValidCMD()) {
            cmd_.sendFeedback("Already at correct X and Y - no need for pathplan");
        }
        return;
    }
    if(cmd_.isValidCMD()) {
        cmd_.sendFeedback("Planning path to target!");
    }
    //Send desired goal to path planner
    geometry_msgs::Point32 destPoint;
    //Only x and y is used
    destPoint.x = static_cast<float>(event.positionGoal.x);
    destPoint.y = static_cast<float>(event.positionGoal.y);

    if(targetPub_.getNumSubscribers() <= 0) {
        fsm.handleFSMError("Planner not listening for target!");
    }

    //Publish target
    targetPub_.publish(destPoint);
    fsm.handleFSMInfo("Sent target and position to planner, waiting for result!");
    

}

void GoToState::stateEnd(ControlFSM& fsm, const EventData& event) {
    isActive_ = false;
}

void GoToState::loopState(ControlFSM& fsm) {
    //Get position
    const geometry_msgs::PoseStamped* pPose = fsm.getPositionXYZ();
    //Should never occur, but just in case
    if(pPose == nullptr) {
        EventData event;
        event.eventType = EventType::POSLOST;
        if(cmd_.isValidCMD()) {
            cmd_.eventError("No position");
            cmd_ = EventData();
        }
        fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, event);
        return;
    }

    //Check if destination is reached!
    double deltaX = pPose->pose.position.x - cmd_.positionGoal.x;
    double deltaY = pPose->pose.position.y - cmd_.positionGoal.y;
    double deltaZ = pPose->pose.position.z - cmd_.positionGoal.z;
    bool xyWithinReach = (std::pow(deltaX, 2) + std::pow(deltaY, 2)) <= std::pow(destReachedMargin_, 2);
    bool zWithinReach = (std::fabs(deltaZ) <= FSMConfig::AltitudeReachedMargin);
    bool yawWithinReach = (std::fabs(fsm.getMavrosCorrectedYaw() - setpoint_.yaw) <= yawReachedMargin_);
    //If destination is reached, begin transition to another state
    if(xyWithinReach && zWithinReach && yawWithinReach) {
        //Hold current position for a duration - avoiding unwanted velocity before doing anything else
        if(!delayTransition_.enabled) {
            delayTransition_.started = ros::Time::now();
            delayTransition_.enabled = true;
            if(cmd_.isValidCMD()) {
                cmd_.sendFeedback("Destination reached, letting drone slow down before transitioning!");
            }
        }
        //Delay transition
        if(ros::Time::now() - delayTransition_.started < delayTransition_.delayTime) {
            return;
        } 
        //Transition to correct state
        if(cmd_.isValidCMD()) {
            switch(cmd_.commandType) {
                case CommandType::LANDXY:
                    fsm.transitionTo(ControlFSM::LANDSTATE, this, cmd_);
                    break;
                /*
                case CommandType::LANDGB:
                    fsm.transitionTo(ControlFSM::TRACKGBSTATE, this, cmd_);
                    break;
                */
                case CommandType::GOTOXYZ:
                    cmd_.finishCMD();
                    RequestEvent doneEvent(RequestType::POSHOLD);
                    fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, doneEvent);
                    break;
            }
        } else {
            RequestEvent posHoldEvent(RequestType::POSHOLD);
            fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, posHoldEvent);
        }
        delayTransition_.enabled = false;
        //Destination reached, no need to excecute the rest of the function
        return;
    } else {
        delayTransition_.enabled = false;
    }

    /**********************************************************/
    //If the destination is not reached, the loop will continue will run

    //Only run pathplanner if neccesary.
    if(xyWithinReach) {
        setpoint_.position.x = cmd_.positionGoal.x;
        setpoint_.position.y = cmd_.positionGoal.y;
        setpoint_.position.z = cmd_.positionGoal.z;
        return;
    } else {
        //Make sure target point is published!!
        if(!safePublisher_.completed) {
            safePublisher_.publish();
        }
        //Send current position to path planner
        geometry_msgs::Point32 currentPos;
        currentPos.x = static_cast<float>(pPose->pose.position.x);
        currentPos.y = static_cast<float>(pPose->pose.position.y);
        posPub_.publish(currentPos);
    }

    //Only continue if there is a valid plan available
    if(!currentPlan_.valid) {
        return;
    } else if(!(bool)currentPlan_.plan.feasibility || currentPlan_.plan.arrayOfPoints.size() <= 0) {
        //A plan is recieved, but there are no points. 
        fsm.handleFSMError("Recieved empty path plan");
        RequestEvent abortEvent(RequestType::ABORT);
        if(cmd_.isValidCMD()) {
            cmd_.eventError("No feasable path to target");
        }
        fsm.transitionTo(ControlFSM::POSITIONHOLDSTATE, this, abortEvent);
        return;
    }

    //Get current setpoint from plan
    auto& currentPoint = currentPlan_.plan.arrayOfPoints[currentPlan_.index]; //geometry_msgs::Point32
    auto& xPos = pPose->pose.position.x;
    auto& yPos = pPose->pose.position.y;
    //Check if we are close enough to current setpoint
    deltaX = xPos - currentPoint.x;
    deltaY = yPos - currentPoint.y;
    if(std::pow(deltaX, 2) + std::pow(deltaY, 2) <= std::pow(setpointReachedMargin_, 2)) {
        //If there are a new setpoint in the plan, change to it.
        if(currentPlan_.plan.arrayOfPoints.size() > (currentPlan_.index + 1)) {
            ++currentPlan_.index;
            currentPoint = currentPlan_.plan.arrayOfPoints[currentPlan_.index];
            deltaX = currentPoint.x - xPos;
            deltaY = currentPoint.y - yPos;
            //Only change yaw if drone needs to travel a large distance
            if(std::pow(deltaX, 2) + std::pow(deltaY, 2) > std::pow(FSMConfig::NoYawCorrectDist, 2)) {
                //-PI_HALF due to mavros bug
                setpoint_.yaw = static_cast<float>(calculatePathYaw(deltaX, deltaY) - PI_HALF);
            }
        }
    }
    //Set setpoint x and y
    setpoint_.position.x = currentPoint.x;
    setpoint_.position.y = currentPoint.y;
}

//Returns valid setpoint
const mavros_msgs::PositionTarget* GoToState::getSetpoint() {
    setpoint_.header.stamp = ros::Time::now();
    return &setpoint_;
}

//New pathplan is recieved
void GoToState::pathRecievedCB(const ascend_msgs::PathPlannerPlan::ConstPtr& msg) {
    //Ignore callback if state is not active
    if(!isActive_) {
        return;
    }
    cmd_.sendFeedback("New path plan recieved!");
    currentPlan_.plan = *msg;
    currentPlan_.valid = true;
    currentPlan_.index = 0;
}

//Initialize state
void GoToState::stateInit(ControlFSM& fsm) {

    //TODO Uneccesary variables - FSMConfig can be used directly
    //Set state variables
    delayTransition_.delayTime = ros::Duration(FSMConfig::GoToHoldDestTime);
    destReachedMargin_ = (float) FSMConfig::DestReachedMargin;
    setpointReachedMargin_ = (float) FSMConfig::SetpointReachedMargin;
    yawReachedMargin_ = (float) FSMConfig::YawReachedMargin;

    //Set all neccesary publishers and subscribers
    posPub_ = fsm.nodeHandler_.advertise<geometry_msgs::Point32>(FSMConfig::PathPlannerPosTopic, 1);
    obsPub_ = fsm.nodeHandler_.advertise<ascend_msgs::PathPlannerPlan>(FSMConfig::PathPlannerObsTopic, 1);
    targetPub_ = fsm.nodeHandler_.advertise<geometry_msgs::Point32>(FSMConfig::PathPlannerTargetTopic , 1);
    planSub_ = fsm.nodeHandler_.subscribe(FSMConfig::PathPlannerPlanTopic, 1, &GoToState::pathRecievedCB, this);
}

//Calculates a yaw setpoints that is a multiple of 90 degrees
//and is as close to the path direction as possible 
//NOTE - method assumes dx and dy is not equal to zero
double GoToState::calculatePathYaw(double dx, double dy) {
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
    } else if(angle > PI/4) {
        angle = PI/2.0;
    } else {
        angle = 0;
    }
    //Invert if dy is negative
    if(dy < 0) {
        angle *= -1;
    }

    return angle;
}

bool GoToState::stateIsReady(ControlFSM &fsm) {
    //TODO set up obstacles stream for planner
    //Skipping check is allowed for debugging
    if(!FSMConfig::RequireAllDataStreams) return true;
    //Makes sure path planner is listening for input
    if(planSub_.getNumPublishers() <= 0) {
        fsm.handleFSMWarn("No path planner publisher");
        return false;    
    } 
    if(targetPub_.getNumSubscribers() <= 0) {
        fsm.handleFSMWarn("No path planner subscriber");
        return false;
    }
    if(posPub_.getNumSubscribers() <= 0) {
        fsm.handleFSMWarn("No path planner subscriber");
        return false;
    }
    if(obsPub_.getNumSubscribers() <= 0) {
        fsm.handleFSMWarn("No path planner subscriber");
        return false;
    }
    return true;
}

void GoToState::handleManual(ControlFSM &fsm) {
    cmd_.eventError("Lost OFFBOARD");
    cmd_ = EventData();
    RequestEvent manualEvent(RequestType::MANUALFLIGHT);
    fsm.transitionTo(ControlFSM::MANUALFLIGHTSTATE, this, manualEvent);
}



