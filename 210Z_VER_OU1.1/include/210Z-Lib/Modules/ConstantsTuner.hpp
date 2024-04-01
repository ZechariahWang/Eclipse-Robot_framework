#include "iostream"

namespace Eclipse {
    class ConstantsTuner {
        public:
            double localKp = 5;
            double localKi= 0;
            double localKd = 35;

            double global_increment_amount = 1;
            double kp_increment_amount = 1;
            double ki_increment_amount = 0.1;
            double kd_increment_amount = 1;

            int CURRENT_MODE = 0; // 0 = kp, 1 = ki, 2 = kd

            bool AngleTurn90;
            bool AngleTurn180;
            bool AngleTurn270;
            bool AngleTurn360;

            void modify_desired_constants();
            void modify_proportional_derivative_increment_amount(double increment_amount);
            void modify_integral_increment_amount(double increment_amount);
            void control_movement_output();
            void shift_constant();
            void display_constant_data();
            void driver_tuner();

    };
}