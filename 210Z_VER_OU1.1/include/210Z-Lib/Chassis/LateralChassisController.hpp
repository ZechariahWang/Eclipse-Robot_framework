#include "iostream"
#include "vector"

namespace Eclipse{
    class Slew{
        private:
            bool init;
        public:
            double enabled; double error; double x_intercept; double y_intercept; double sign; double slope; double max_speed; double slew_ticks_per_inch;

            std::vector<double> min_power;
            std::vector<double> max_distance;

            /**
             * @brief Initialize slew data logic
             * 
             * @param slew_enabled Is the slew controller enabled
             * @param max_speec Max speed of slew
             * @param target_pos target end of slew
             * @param current_pos current position of slew
             * @param start init location of slew
             * @param backwards_enabled are we going backwards?
             * @param tpi ticks per inch
             */

            void initialize_slew(bool slew_enabled, const double max_speed, const double target_pos, const double current_pos, const double start, bool backwards_enabled, double tpi);

            /**
             * @brief Slew init local data variables
             * 
             */

            void set_slew_min_power(std::vector<double> min_power);

            /**
             * @brief Slew init local data variables
             * 
             */

            void set_slew_distance(std::vector<double> distance);

            /**
             * @brief calculate slew max voltage request
             * 
             * @param current Current position of slew
             * @return Slew speed
             */

            double calculate_slew(const double current);
    };

    class TranslationPID{
        private:
            bool init;
        public:
            double t_kp; double t_ki; double t_kd; double t_h_kp; double t_error; double t_prev_error; double t_integral; double t_derivative; double t_error_thresh; double t_iterator; double t_tol; double t_failsafe; double t_maxSpeed;
            double wheelDiameter= 0; double ratio= 0; double cartridge= 0; double circumference = 0; double ticks_per_rev= 0; double ticks_per_inches= 0;

            /**
             * @brief PID class constructors
             * 
             */

            TranslationPID();

            /**
             * @brief PID class constants
             * 
             */

            void reset_t_alterables();

            /**
             * @brief PID class constants
             * 
             */

            void set_t_constants(const double kp, const double ki, const double kd, const double r_kp);

            /**
             * @brief Set drivetrain specs
             * 
             * @param n_wheelDiameter the dimater of the drivetrain wheel
             * @param n_gearRatio The gear ratio of the drivetrain
             * @param n_motorCartidge The motor cartridge of the drivetrain
             */

            void set_dt_constants(const double n_wheelDiameter, const double n_gearRatio, const double n_motorCartridge);

            /**
             * @brief Driver PID function. Main logic function, combining all translation PID components together
             * 
             * @param target the target translation target position
             * @param maxSpeed the maxspeed the robot may travel at
             */

            void set_translation_pid(double target, double max_speed, bool slew_enabled);

            /**
             * @brief calculate the min angle needed to reach a target theta within 360 degrees
             * 
             * @param targetHeading the target heading of the robot
             * @param currentrobotHeading the current angle held by the robot
             * @return the shortest turn angle needed to reach target theta
             */

            double find_min_angle(int16_t targetHeading, int16_t currentrobotHeading);

            /**
             * @brief compute translation PID movement logic
             * 
             * @param current the current position of the robot
             * @param target the desired target of the robot
             * @return PID calculated voltage at that specific position relative to target
             */

            double compute_t(double current, double target);
    };

    class RotationPID{
        private:
            bool init;
        public:
            double r_kp; double r_ki; double r_kd; double r_error; double r_prev_error; double r_integral; double r_derivative; double r_error_thresh = 3; double r_iterator; double r_tol = 10; double r_failsafe; double r_maxSpeed;

            /**
             * @brief PID class constructors
             * 
             */

            RotationPID();

            /**
             * @brief PID class alterables
             * 
             */

            void reset_r_alterables();

            /**
             * @brief PID class constants
             * 
             */

            void set_r_constants(const double kp, const double ki, const double kd);

            /**
             * @brief compute rotation PID movement logic
             * 
             * @param current the current raw IMU value of the robot
             * @param target the desired target theta of the robot (IN RAW IMU VALUES)
             * @return PID calculated voltage at that specific angle relative to target angle
             */

            double compute_r(double current, double target);

            /**
             * @brief Driver Rotation PID function. Main logic function, combining all rotation PID components together
             * 
             * @param t_theta the target theta angle
             * @param maxSpeed the maxspeed the robot may make the turn in 
             */

            void set_rotation_pid(double t_theta, double max_speed);
    };

    class CurvePID{
        private:
            bool init;
        public:
            double c_kp; double c_ki; double c_kd; double c_error; double c_prev_error; double c_integral; double c_derivative; double c_error_thresh = 3; double c_iterator; double c_tol = 10; double c_failsafe; double c_maxSpeed; bool c_rightTurn;

            /**
             * @brief PID class constructors
             * 
             */

            CurvePID();

            /**
             * @brief PID class alterables
             * 
             */

            void reset_c_alterables();

            /**
             * @brief PID class constants
             * 
             */

            void set_c_constants(const double kp, const double ki, const double kd);

            /**
             * @brief compute curve PID movement logic
             * 
             * @param current the current raw IMU value of the robot
             * @param target the desired target theta the robot will curve to
             * @return PID calculated voltage at that specific angle relative to target curve
             */

            double compute_c(double current, double target);

            /**
             * @brief Driver Curve PID function. Main logic function, combining all curve PID components together
             * 
             * @param t_theta the target theta angle
             * @param maxSpeed the maxspeed the robot may make the turn in 
             * @param curveDamper The amount the swing will be dampered by
             * @param backwards whether or not the robot will make the curve backwards or forwards
             */

            void set_curve_pid(double t_theta, double max_speed, double curve_damper, bool backwards);
    };

    class ArcPID{
        private:
            bool init;
        public:
            double a_kp; double a_ki; double a_kd; double a_error; double a_prev_error; double a_integral; double a_derivative; double a_error_thresh = 3; double a_iterator; double a_tol = 10; double a_failsafe; double a_maxSpeed; bool a_rightTurn;

            /**
             * @brief PID class constructors
             * 
             */

            ArcPID();

            /**
             * @brief PID class alterables
             * 
             */

            void reset_a_alterables();

            /**
             * @brief PID class constants
             * 
             */

            void set_a_constants(const double kp, const double ki, const double kd);

            /**
             * @brief compute arc PID movement logic
             * 
             * @param tx the target global X value calculated from Odometry logic
             * @param ty the target global Y value calculated from Odometry logic
             * @return PID calculated voltage at that specific coordinate vector relative to target vector
             */

            double compute_a(double tx, double ty);

            /**
             * @brief Driver Arc PID function. Main logic function, combining all arc PID components together
             * 
             * @param t_x the target X position coordinate
             * @param t_y the target Y position coordinate
             * @param maxSpeed the max speed the robot may make the turn in 
             * @param arcDamper the amount the arc movement will be dampered by
             */

            void set_arc_pid(double t_x, double t_y, double max_speed, double arc_damper);
    };
}
