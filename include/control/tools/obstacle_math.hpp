#include <cmath>

namespace ObstacleMath {

constexpr double PI{3.14159265358979323846264338327950288419};

// wrap any angle to range [0, 2pi)
template<typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
T angleWrapper(const T angle){
    return angle - 2*PI*floor(angle/(2*PI));
}

/// Calculate vector between two points, end - start
template<typename T, typename K>
inline T vectorDifference(const T& end, const K& start){
    T vector_diff;
    vector_diff.x = end.x - start.x;
    vector_diff.y = end.y - start.y;
    vector_diff.z = end.z - start.z;

    return vector_diff;
}


template<typename T, typename K>
inline T vectorSum(const T& a, const K& b){
    T vector_sum;
    vector_sum.x = a.x + b.x;
    vector_sum.y = a.y + b.y;
    vector_sum.z = a.z + b.z;

    return vector_sum;
}

template<typename T, typename K>
inline double calcDistanceToObstacle(const T& point, const K& obstacle_position){
    const auto vector_to_obstacle = vectorDifference(point, obstacle_position);
    const double distance_to_obstacle = std::sqrt(std::pow(vector_to_obstacle.x, 2) + std::pow(vector_to_obstacle.y, 2));

    return distance_to_obstacle;
}


template<typename T, typename K, typename N>
inline N calcAngleToObstacle(const T& point, const K& obstacle_position, const N obstacle_direction){
    T delta_drone_obstacle;
    delta_drone_obstacle.x = point.x - obstacle_position.x;
    delta_drone_obstacle.y = point.y - obstacle_position.y;

    const float angle_to_obstacle = angleWrapper(std::atan2(delta_drone_obstacle.y, delta_drone_obstacle.x) - obstacle_direction);

    return angle_to_obstacle;
}

// Apply 2d transformation matrix
template<typename T>
inline T rotateXY(const T& point, const float angle){
    T new_point;
    new_point.x = point.x * std::cos(angle) - point.y * std::sin(angle);
    new_point.y = point.x * std::sin(angle) + point.y * std::cos(angle);
    new_point.z = point.z;

    return new_point;
}

}
