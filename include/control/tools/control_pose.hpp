#ifndef CONTROL_POSE
#define CONTROL_POSE
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
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
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
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
