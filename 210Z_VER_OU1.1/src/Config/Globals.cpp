/**
 * @file Globals.cpp
 * @author Zechariah Wang
 * @brief Robot physical parts declaration
 * @version 0.1
 * @date 2023-07-04
 */

#include "Globals.hpp"
#include "pros/adi.hpp"
#include "main.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Holonomic drive dedicated motors. Not normally used
pros::Motor dt_front_left(19, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor dt_front_right(16, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor dt_rear_left(7, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor dt_rear_right(5, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor dt_middle_left(9, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor dt_middle_right(40, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::MotorGroup left_side_motors({dt_front_left, dt_middle_left, dt_rear_left});
pros::MotorGroup right_side_motors({dt_front_right, dt_middle_right, dt_rear_right});

// Lateral PID Classes
Eclipse::TranslationPID mov_t; Eclipse::RotationPID rot_r; Eclipse::CurvePID cur_c; Eclipse::ArcPID arc_a;  Eclipse::FeedbackControl mtp;

// Position Tracking
Eclipse::Odometry odom; Eclipse::KalmanFilter kal;

// Match Control
Eclipse::MatchMovement op_mov;

// Misc
Eclipse::Metrics data_displayer; Eclipse::Selector data; Eclipse::Slew slew; Eclipse::Math math;

