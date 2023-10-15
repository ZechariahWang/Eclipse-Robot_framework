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

bool GPS_ENABLED;
pros::ADIEncoder vertical_auxiliary_sensor('d', 'f', true);
pros::Rotation horizontal_rotation_sensor(8);
pros::Gps gps_sensor(99);
pros::c::gps_status_s_t gpsData;
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Imu imu_sensor(20);
pros::Motor dt_front_left(config.front_left_port, pros::E_MOTOR_GEARSET_06, config.left_reversed, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor dt_front_right(config.front_right_port, pros::E_MOTOR_GEARSET_06, config.right_reversed, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor dt_rear_left(config.rear_right_port, pros::E_MOTOR_GEARSET_06, config.left_reversed, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor dt_rear_right(config.rear_right_port, pros::E_MOTOR_GEARSET_06, config.right_reversed, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor dt_middle_left(config.middle_left_port, pros::E_MOTOR_GEARSET_06, config.left_reversed, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor dt_middle_right(config.middle_right_port, pros::E_MOTOR_GEARSET_06, config.right_reversed, pros::E_MOTOR_ENCODER_COUNTS);
pros::MotorGroup left_side_motors({dt_front_left, dt_middle_left, dt_rear_left});
pros::MotorGroup right_side_motors({dt_front_right, dt_middle_right, dt_rear_right});

Eclipse::TranslationPID mov_t;
Eclipse::RotationPID rot_r;
Eclipse::CurvePID cur_c;
Eclipse::ArcPID arc_a; 
Eclipse::FeedbackControl mtp;
Eclipse::Odometry odom;
Eclipse::MatchMovement op_mov;

Eclipse::Metrics data_displayer;
Eclipse::Selector data; 

Eclipse::KalmanFilter kal;
Eclipse::Slew slew;
Eclipse::Math math;

pros::ADIAnalogIn cata_sensor('h');
pros::Motor cata_motor(3, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor intake_motor(12, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

