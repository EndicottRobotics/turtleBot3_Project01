#ifndef UTILS_H
#define UTILS_H

// Include necessary libraries
#include <cmath>

// Constant for PI, already defined in the cpp file, no need to define again here.
extern const double PI; 

// Function declarations
double unwrap(double angle);
double cal_relative_polar_distance(double target_x, double target_y, double current_x, double current_y);
double cal_relative_polar_angle(double target_x, double target_y, double current_x, double current_y);

#endif // UTILS_H