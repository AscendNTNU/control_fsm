#include "control_fsm/tools/target_tools.hpp"
#include <cmath>

///Calculate constants at compiletime
constexpr double PI = 3.14159265359;
constexpr double PI_HALF = PI / 2.0;
constexpr double MAVROS_CORRECTION_PI_HALF = PI_HALF;

double control::getMavrosCorrectedTargetYaw(double current) {
    //Get angle in interval [-pi, pi]
    double theta = std::atan2(std::sin(current), std::cos(current));
    if (theta > 3.0 * PI / 4.0) theta = PI;
    else if (theta > PI / 4.0) theta = PI_HALF;
    else if (theta > -PI / 4.0) theta = 0.0;
    else if (theta > -3.0 * PI / 4) theta = -PI_HALF;
    else theta = -PI;
    //Remove pi half to correct for 
    return theta - MAVROS_CORRECTION_PI_HALF;
}
