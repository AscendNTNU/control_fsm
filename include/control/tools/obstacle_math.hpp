#ifndef CONTROL_OBSTACLE_MATH_HPP_
#define CONTROL_OBSTACLE_MATH_HPP_
#include <type_traits>

template<typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
T angleWrapper(const T angle);

template<typename T, typename K>
inline T vectorDifference(const T& point_A, const K& point_B);

template<typename T, typename K>
inline float calcDistanceToObstacle(const T& point, const K& obstacle_position);

template<typename T, typename K, typename N>
inline N calcAngleToObstacle(const T& point, const K& obstacle_position, const N obstacle_direction);

template<typename T>
inline T rotateXY(const T& point, const float angle);


#endif //CONTROL_MATH_HPP_

