#ifndef INTERCEPT_GROUNDBOT_HPP
#define INTERCEPT_GROUNDBOT_HPP

#include "control/tools/intercept_groundbot.hpp"

namespace control {
namespace gb {
/** Calculate setpoint to intercept a ground robot
 * @param quad_position Drones current pose
 * @param roomba_position Tracked ground robot
 * @setpoint Setpoint to be modified
 * @return true if setpoint is modified (all ok), false if not
 */
bool interceptGB(const geometry_msgs::PoseStamped& quad_position, 
                 const ascend_msgs::GRState&       roomba_position, 
                 mavros_msgs::PositionTarget&      setpoint); 
}
}


#endif
