#ifndef INTERCEPT_GROUNDBOT_HPP
#define INTERCEPT_GROUNDBOT_HPP

#include "control/tools/intercept_groundbot.hpp"


bool InterceptGB(geometry_msgs::PoseStamped quad_position, ascend_msgs::GRState roomba_position, mavros_msgs::PositionTarget& setpoint); 



#endif
