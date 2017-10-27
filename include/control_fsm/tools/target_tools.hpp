#ifndef TARGET_TOOLS_HPP
#define TARGET_TOOLS_HPP

namespace control {

/** 
 * Returns closest multiplum of 90 degrees to input yaw
 * Also corrects for mavros yaw bug
*/
double getMavrosCorrectedTargetYaw(double current);
    
}

#endif
