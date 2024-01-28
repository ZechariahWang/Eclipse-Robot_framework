#include "main.h"
#include <vector>

using namespace Eclipse;

double time_to_target_vel;
double distance_to_target_vel;

double global_distance;
double global_acceleration;
double global_max_velocity;
double global_init_pos;

void trapezoidal_reset_values(){
    time_to_target_vel = 0;
    distance_to_target_vel = 0;

    global_distance = 0;
    global_acceleration = 0;
    global_max_velocity = 0;
    global_init_pos = 0;
}

void trapezoidal_calculate_initial_kinematic_values(double targetPosition, double maxVelocity, double acceleration) {
    utility::restart_all_chassis_motors(false);
    double initialPosition = utility::get_encoder_position();
    double distance = targetPosition - initialPosition;
    double targetVelocity = utility::sgn(distance) * maxVelocity;
    
    // Calculate the time to reach the target velocity
    double timeToTargetVelocity = targetVelocity / acceleration;

    // Calculate the distance to accelerate to the target velocity
    double distanceToTargetVelocity = 0.5 * acceleration * timeToTargetVelocity * timeToTargetVelocity;

    time_to_target_vel = timeToTargetVelocity;
    distance_to_target_vel = distanceToTargetVelocity;

    global_distance = distance;
    global_acceleration = acceleration;
    global_max_velocity = maxVelocity;
    global_init_pos = initialPosition;
}

void trapezoidal_calculate_motion_profile(){
    if (fabs(global_distance) > 2 * distance_to_target_vel) {
        // Trapezoidal profile
        double timeToDeacceleration = fabs(global_distance) / global_max_velocity;
        double totalTime = time_to_target_vel + timeToDeacceleration;

        double accelerationTimePercentage = time_to_target_vel / totalTime;
        double decelerationTimePercentage = timeToDeacceleration / totalTime;

        // Perform the motion profile
        double startTime = 0;
        double currentTime;
        double currentPosition;

        while (true) {
            currentTime = (pros::millis() - startTime) / 1000;
            if (currentTime >= totalTime) {
                utility::motor_deactivation();
                break;
            } else if (currentTime <= time_to_target_vel) {
                // Acceleration phase
                currentPosition = global_init_pos + utility::sgn(global_distance) * 0.5 * global_acceleration * currentTime * currentTime;
                utility::engage_left_motors(utility::sgn(global_distance) * global_acceleration * currentTime);
                utility::engage_right_motors(utility::sgn(global_distance) * global_acceleration * currentTime);
            } else if (currentTime <= totalTime - timeToDeacceleration) {
                // Constant velocity phase
                currentPosition = global_init_pos + utility::sgn(global_distance) * (distance_to_target_vel + global_max_velocity * (currentTime - time_to_target_vel));
                utility::engage_left_motors(utility::sgn(global_distance) * global_max_velocity);
                utility::engage_right_motors(utility::sgn(global_distance) * global_max_velocity);
            } else {
                // Deceleration phase
                double decelerationTime = currentTime - (totalTime - timeToDeacceleration);
                currentPosition = global_init_pos + utility::sgn(global_distance) * (global_distance - 0.5 * global_acceleration * decelerationTime * decelerationTime);
                utility::engage_left_motors(utility::sgn(global_distance) * (global_max_velocity - global_acceleration * decelerationTime));
                utility::engage_right_motors(utility::sgn(global_distance) * (global_max_velocity - global_acceleration * decelerationTime));
            }
            pros::delay(10); 
        }
    } else {
        // Triangular profile (no constant velocity phase)
        double accelerationTime = sqrt(fabs(global_distance) / global_acceleration);
        double totalTime = 2 * accelerationTime;

        // Perform the motion profile
        double startTime = 0;
        double currentTime;
        double currentPosition;

        while (true) {
            currentTime = (pros::millis() - startTime) / 1000;
            if (currentTime >= totalTime) {
                utility::motor_deactivation();
                break;
            } else if (currentTime <= accelerationTime) {
                // Acceleration phase
                currentPosition = global_init_pos + utility::sgn(global_distance) * 0.5 * global_acceleration * currentTime * currentTime;
                utility::engage_left_motors(utility::sgn(global_distance) * global_acceleration * currentTime);
                utility::engage_right_motors(utility::sgn(global_distance) * global_acceleration * currentTime);
            } else {
                // Deceleration phase
                double decelerationTime = currentTime - accelerationTime;
                currentPosition = global_init_pos + utility::sgn(global_distance) * (global_distance - 0.5 * global_acceleration * decelerationTime * decelerationTime);
                utility::engage_left_motors(utility::sgn(global_distance) * (global_max_velocity - global_acceleration * decelerationTime));
                utility::engage_right_motors(utility::sgn(global_distance) * (global_max_velocity - global_acceleration * decelerationTime));
            }

            pros::delay(10); 
        }
    }
}

double calculate_trapezoidal_velocity(int distance, int accelDistance, int maxDistance, int initialVel, int maxVel, int acceleration, int deacceleration){
    double velocity;
    if (distance < accelDistance) {
        velocity = sqrt(initialVel * initialVel + 2 * acceleration * distance);
    }
    else if (distance < maxDistance - accelDistance) {
        velocity = maxVel;
    }
    else {
        velocity = double((maxVel * maxVel)) / (2 * distance);
    }

    return velocity;
}

void trapezoidal_driver(double target_pos) {
    int initialPosition = 0; // Initial position
    int targetPosition = target_pos; // Target position
    int maxVelocity = 70; // Maximum velocity
    int maxAcceleration = 10; // Maximum acceleration
    int maxDeAcceleration = 20;
    int accelerationDistance = (maxVelocity * maxVelocity) / (2 * maxAcceleration);

    int currentPosition = initialPosition;
    int currentTime = 0;
    int dt = 10; // Time step in milliseconds

    utility::restart_all_chassis_motors(false);

    while (currentPosition < targetPosition) {
        int currentVelocity = calculate_trapezoidal_velocity(currentPosition, accelerationDistance, targetPosition, 20, maxVelocity, maxAcceleration, maxDeAcceleration);

        utility::engage_left_motors(currentVelocity * (12000.0 / 127));
        utility::engage_right_motors(currentVelocity * (12000.0 / 127));
        currentPosition = utility::get_encoder_position();

        // Increment time
        currentTime += dt;
        pros::delay(dt);
    }

    utility::motor_deactivation();
}


double distance = 0;
double position = 0;
double velocity = 0;
double max_acceleration = 0;
double max_velocity = 0;
double current_time = 0;

std::vector<double> timePhase = {0,0,0};
std::vector<double> distancePhase;
std::vector<double> velocityPhase;


double get_current_position() { return position; }
void set_constraints() {}
void trapezoidal_motion_profiler() { timePhase[0] = 5; }
void move_with_trapezoidal(double distance, double timeout) {}

