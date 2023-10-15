#include "iostream"
#include "api.h"

class PID{
    public:
        double kp = 0;
        double ki = 0;
        double kd = 0;
        double main_threshold = 0;
        double main_tolerance = 0;
        double failsafe_threshhold = 0;
        double failsafe_tolerance = 0;
        double max_speed = 0;

        double output = 0;
        double error = 0;
        double integral = 0;
        double derivative = 0;
        double previous_error = 0;
        double iterator = 0;
        double failsafe = 0;

        PID(double kp, double ki, double kd, double threshold, double tolerance, double failsafe_threshhold, double failsafe_tolerance, double max_speed);
        void reset_values();
        double compute_pid(double current, double target);
        void set_pid(pros::Motor &reference_motor, double target);
};