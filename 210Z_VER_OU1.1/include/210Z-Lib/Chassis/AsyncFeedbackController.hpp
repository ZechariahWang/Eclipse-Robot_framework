#include "iostream"

namespace Eclipse{
    class FeedbackControl{
        private:
            bool init_MotionAlg;
        public:
            double target_tol; double target_final_tol; double t_kp; double r_kp; double iterator;
            double distance; double alpha; double beta; double t_error; double r_error;
            double previous_drive_error; double previous_turn_error;
            double t_local_kp = 5; double t_local_ki = 0.003; double t_local_kd = 35;
            double t_local_derivative;  double t_local_integral;  double t_local_tolerance; double t_local_error; double t_local_previouserror; double t_local_multiplier;
            double t_local_averageposition; double t_local_averageHeading; double t_local_FailSafeCounter; int t_local_threshholdcounter;

            double lkp = 0;
            double lkd = 0;
            double akp = 0;
            double akd = 0;
            double mtp_max_linear_speed = 0;
            double mtp_max_angular_speed = 0;

            FeedbackControl();
            void set_constants(double t_kp, double r_kp, double f_tt, double t); 
            void reset_mtp_constants();
            void reset_swing_alterables();
            void TurnToPoint(int targetX, int targetY);
            void move_to_reference_pose(double tx, double ty, double targetHeading, double radius);
            void swing_to_point(double tx, double ty, double swingDamper);
            void overRideCoordinatePos(double new_gx, double new_gy);
            void simultaneous_mov_executor(double targetX, double targetY, double targetTheta, double translationSpeed, double rotationSpeed);
            void set_mtp_constants(double lkp, double lkd, double akp, double akd, double max_linear_speed, double max_angular_speed);
            void move_to_point(double target_x, double target_y, bool backwards, bool init_delay, int timer);
            void boomerang(double target_x, double target_y, double target_theta, double d_lead, bool reverse);
    };

}

void mimic_move_to_point(double target_x, double target_y, bool reverse);

