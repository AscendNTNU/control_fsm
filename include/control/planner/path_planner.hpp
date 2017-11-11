#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <ros/ros.h>
#include <ascend_msgs/PathPlannerPlan.h>
#include "control/tools/control_pose.hpp"


class PathPlanner{
private:
    
ascend_msgs::PathPlannerPlan current_plan_;

public:
    PathPlanner();
    
    
    
};

#endif // PATH_PLANNER_HPP
