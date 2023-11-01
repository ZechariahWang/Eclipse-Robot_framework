#include "api.h"

void trapezoidal_reset_values();
void trapezoidal_calculate_initial_kinematic_values(double targetPosition, double maxVelocity, double acceleration);
void trapezoidal_calculate_motion_profile();


double calculate_trapezoidal_velocity();
void trapezoidal_driver(double target_pos);
