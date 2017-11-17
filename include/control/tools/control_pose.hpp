#ifndef CONTROL_POSE
#define CONTROL_POSE
#include <cmath>
namespace control {
namespace pose {
//Anonymous namespace - only available in this file
namespace {
    constexpr double MAVROS_YAW_OFFSET = 1.57079632679;
}
/**Calculates yaw based on quaternion orientation
 *\param T Must contain x, y, z and w public members
 */
template<typename T>
double quat2yaw(T q) {
    double siny = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
    return std::atan2(siny, cosy);
}

/**Calculates yaw based on quaternion and corrects for mavros bug
 * \param T Must contain x, y, z and w public members
 */
template<typename T>
double quat2mavrosyaw(T q) {
    return quat2yaw(q) - MAVROS_YAW_OFFSET;
}
}
}
#endif
