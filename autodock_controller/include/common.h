#include <cmath>
#include <boost/array.hpp>
#include <iostream>

template <typename T>
inline double distance(T robot_point, T last_robot_point){
    
    double dist = sqrt(pow(fabs(robot_point[0] - last_robot_point[0]),2)+pow(fabs(robot_point[1] - last_robot_point[1]),2));
    
    return dist;
}