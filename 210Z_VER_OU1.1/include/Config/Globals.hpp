#include "pros/adi.hpp"

#pragma once
#include "main.h"

extern pros::Controller controller;
extern pros::Motor dt_front_left;
extern pros::Motor dt_front_right;
extern pros::Motor dt_middle_left;
extern pros::Motor dt_middle_right;
extern pros::Motor dt_rear_left;
extern pros::Motor dt_rear_right;

extern pros::ADIEncoder vertical_auxiliary_sensor;
extern pros::Rotation horizontal_rotation_sensor;
extern pros::Imu imu_sensor;


extern pros::MotorGroup left_side_motors;
extern pros::MotorGroup right_side_motors;

extern pros::ADIAnalogIn cata_sensor;
extern pros::Motor cata_motor;
extern pros::Motor cata_motor_secondary;
extern pros::Motor intake_motor;
extern pros::Distance distance_sensor;

extern pros::ADIDigitalOut left_wing;
extern pros::ADIDigitalOut right_wing;
extern pros::ADIDigitalOut odom_piston;
extern pros::ADIDigitalOut blocker;
extern pros::ADIDigitalOut climber;

extern Eclipse::KalmanFilter kal;
extern Eclipse::TranslationPID mov_t;
extern Eclipse::MatchMovement op_mov;
extern Eclipse::Metrics data_displayer;
extern Eclipse::Selector data; 
extern Eclipse::Slew slew;
extern Eclipse::RotationPID rot_r;
extern Eclipse::CurvePID cur_c;
extern Eclipse::ArcPID arc_a; 
extern Eclipse::FeedbackControl mtp;
extern Eclipse::Odometry odom;
extern Eclipse::Math math;
