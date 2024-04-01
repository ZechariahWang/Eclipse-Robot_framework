/**
 * @file AsyncFeedbackController.hpp
 * @author Zechariah Wang
 * @brief Header file for controller
 * @version 0.1
 * @date 2024-03-28
 * 
 */

#include "iostream"

namespace Eclipse{
    class FeedbackControl{
        private:
            bool init_MotionAlg;
        public:

            /**
            * @brief local PID modifiers
            * 
            */

            double target_tol; double target_final_tol; double t_kp; double r_kp; double iterator;
            double distance; double alpha; double beta; double t_error; double r_error;
            double previous_drive_error; double previous_turn_error;
            double t_local_kp = 5; double t_local_ki = 0.003; double t_local_kd = 35;
            double t_local_derivative;  double t_local_integral;  double t_local_tolerance; double t_local_error; double t_local_previouserror; double t_local_multiplier;
            double t_local_averageposition; double t_local_averageHeading; double t_local_FailSafeCounter; int t_local_threshholdcounter;

            /**
            * @brief PID Constants
            * 
            * @param n_wheelDiameter the dimater of the drivetrain wheel
            * @param n_gearRatio The gear ratio of the drivetrain
            * @param n_motorCartidge The motor cartridge of the drivetrain
            */

            double lkp = 0;
            double lkd = 0;
            double akp = 0;
            double akd = 0;
            double mtp_max_linear_speed = 0;
            double mtp_max_angular_speed = 0;

            /**
            * @brief Constructor for Feedback Controller
            * 
            */

            FeedbackControl();

            /**
            * @brief Set constants
            * 
            * @param t_kp the kp of the controller
            * @param r_kp the kp of the rotation controller
            */
            void set_constants(double t_kp, double r_kp, double f_tt, double t); 
            void reset_mtp_constants();
            void reset_swing_alterables();

            /**
             * @brief Turn to a specific coordinate position
             * 
             * @param targetX the target x coordinate
             * @param targetY the target y coordinate
             */     

            void TurnToPoint(int targetX, int targetY);

            /**
             * @brief move to a specific position at a specific angle
             * 
             * @param targetX the target x coordinate
             * @param targetY the target y coordinate
             * @param targetHeading the target angle in degrees
             * @param radius the radius of the arc
             */

            void move_to_reference_pose(double tx, double ty, double targetHeading, double radius);
            void overRideCoordinatePos(double new_gx, double new_gy);

            /**
             * @brief MTP Algorithm. Move to a desired coordinate position while facing a desired angle. Can only be used with mecanum drives
             * 
             * @param targetX the target x coordinate
             * @param targetY the target y coordinate
             * @param targetTheta the target angle in degrees
             * @param translationSpeed max movement speed
             * @param rotationSpeed max rotation speed
             */ 


            void simultaneous_mov_executor(double targetX, double targetY, double targetTheta, double translationSpeed, double rotationSpeed);
            void set_mtp_constants(double lkp, double lkd, double akp, double akd, double max_linear_speed, double max_angular_speed);

            /**
             * @brief MTP Algorithm. Move to a desired coordinate position while facing a desired angle. Can only be used with mecanum drives
             * 
             * @param targetX the target x coordinate
             * @param targetY the target y coordinate
             * @param targetTheta the target angle in degrees
             * @param translationSpeed max movement speed
             * @param rotationSpeed max rotation speed
             */ 

            void move_to_point(double target_x, double target_y, bool backwards, bool init_delay, double timer);
            void boomerang(double target_x, double target_y, double target_theta, double d_lead, bool reverse);
    };

}

void mimic_move_to_point(double target_x, double target_y, bool reverse);

