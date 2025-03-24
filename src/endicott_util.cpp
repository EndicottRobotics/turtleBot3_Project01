#include "../include/endicott_util.hpp"


const double PI = M_PI; // Use M_PI for the value of pi

// Unwrap function to keep the angle between -π and π
double unwrap(double angle) {
    while (angle > PI) {
        angle -= 2 * PI;  // Subtract 2π to bring the angle within the range
    }
    while (angle < -PI) {
        angle += 2 * PI;  // Add 2π to bring the angle within the range
    }
    return angle;
}

double cal_relative_polar_distance( double x1, double y1, double x2, double y2){
    return (std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2)));
  }

double cal_relative_polar_angle( double x1, double y1, double x2, double y2){
    return( std::atan2(y1 - y2, x1 - x2));
  }

