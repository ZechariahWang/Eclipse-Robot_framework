/**
 * @file WorldsPath.cpp
 * @author Zechariah Wang
 * @brief All paths for Worlds 2024
 * @version 0.1
 * @date 2024-03-28
 */

#include "Config/Globals.hpp"
#include "main.h"

using namespace Eclipse;

void intake(double speed) { intake_motor.move_voltage(-speed); intake_motor_secondary.move_voltage(speed); }

void Eclipse::Paths::local_close_side() {

    intake(-12000);
    odom_piston.set_value(false);
    right_front_wing.set_value(true);

    rot_r.set_r_constants(5, 0, 45);
    rot_r.set_rotation_pid(19, 90, 0.4);

    intake(12000); // intake
    right_front_wing.set_value(false);

	mov_t.set_t_constants(8, 0, 45, 600);
	mov_t.set_translation_pid(60, 110, 3, false);

    pros::delay(200);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(-24, 110, 3, false);

    cur_c.set_c_constants(7, 0, 45);
    cur_c.set_curve_pid(90, 90, 0.1, 1, true);

	mov_t.set_t_constants(6, 0, 45, 300);
	mov_t.set_translation_pid(-38, 90, 1, false);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(0, 90, 3);

    cur_c.set_c_constants(7, 0, 45);
    cur_c.set_curve_pid(-45, 110, 0.28, 2, true);

    left_wing.set_value(true);
    pros::delay(200);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(-23, 90, 1, false);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(-90, 90, 1);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(-45, 90, 1);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(10, 70, 1, false);

    left_wing.set_value(false);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(-27, 70, 1, false);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(0, 90, 3);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(90, 90, 3);

    intake_motor.move_voltage(12000);
    intake_motor_secondary.move_voltage(-12000);
    right_front_wing.set_value(true);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(40, 30, 5, true);

    right_front_wing.set_value(false);
}

void Eclipse::Paths::local_six_ball() {

    intake(-12000);
    pros::delay(200);
    intake(12000);
    odom_piston.set_value(false);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(10, 120, 3, false);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(0, 120, 1);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(-33, 120, 3, false);

    cur_c.set_c_constants(7, 0, 45);
    cur_c.set_curve_pid(-45, 120, 0.18, 2, true);

    left_wing.set_value(true);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(-24, 120, 3, false);

    left_wing.set_value(false);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(-75, 120, 1);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(-22, 120, 0.7, false);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(10, 120, 1, false);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(110, 120, 1);

    intake(-12000);

	mov_t.set_t_constants(8, 0, 45, 600);
	mov_t.set_translation_pid(27, 120, 0.8, false);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(-20, 120, 0.7, false);

    intake(12000);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(35, 120, 1);

    mov_t.set_t_constants(7, 0, 35, 500);
	mov_t.set_translation_pid(40, 120, 4, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(0, 120, 1);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(20, 120, 0.7, false);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(0, 120, 1);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-12, 120, 2, false);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(145, 120, 3);

    intake(-12000);
    pros::delay(500);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(54, 120, 3);

    intake(12000);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(35, 127, 1, false);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(180, 120, 3);

    left_front_wing.set_value(true);
    right_front_wing.set_value(true);
    intake(-12000);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(48, 127, 1, false);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(180, 120, 0.5);

    left_front_wing.set_value(false);
    right_front_wing.set_value(false);

    cur_c.set_c_constants(7, 0, 45);
    cur_c.set_curve_pid(90, 110, 0.28, 2, true);

}

void Eclipse::Paths::global_close_side() {

}

void Eclipse::Paths::global_six_ball() {

    intake(12000);

	mtp.set_mtp_constants(9, 45, 7, 35, 120, 90);
	mtp.move_to_point(39, 12, false, false, 1.3);

	mtp.set_mtp_constants(9, 45, 5, 35, 120, 90);
	mtp.move_to_point(0, 0, true, false, 1.1);

    intake(-12000);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(45, 120, 0.8);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(-85, 120, 3);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(45, 120, 3, true);

    rot_r.set_r_constants(5, 0, 45);
    rot_r.set_rotation_pid(-90, 120, 2);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(-37, 120, 3, true);

    cur_c.set_c_constants(7, 0, 45);
    cur_c.set_curve_pid(-135, 120, 0.18, 2, true);

    left_wing.set_value(true);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(-27, 120, 3, true);

    left_wing.set_value(false);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(-170, 120, 1);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(-22, 120, 1, true);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(8, 120, 0.5, true);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(10, 120, 1.5);

    intake(-12000);

	mov_t.set_t_constants(8, 0, 45, 600);
	mov_t.set_translation_pid(24, 120, 0.7, true);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(-13, 120, 0.7, true);

    intake(12000);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(-65, 120, 3);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(57, 120, 4, true);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-90, 120, 1);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(50, 120, 1);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(34, 120, 4, true);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 120, 1);

    intake(-12000);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(30, 120, 1, true);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-15, 120, 1, true);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-90, 120, 1);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(30, 120, 1, true);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 120, 1);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(30, 120, 1, true);


    // rot_r.set_r_constants(6, 0, 45);
    // rot_r.set_rotation_pid(45, 120, 1);

    // intake(-12000);

    // mov_t.set_t_constants(5, 0, 35, 500);
	// mov_t.set_translation_pid(30, 120, 4, true);

    // mov_t.set_t_constants(5, 0, 35, 500);
	// mov_t.set_translation_pid(-15, 120, 4, true);

    // rot_r.set_r_constants(6, 0, 45);
    // rot_r.set_rotation_pid(-90, 120, 1);


    // intake(12000);

	// mov_t.set_t_constants(5, 0, 45, 600);
	// mov_t.set_translation_pid(32, 127, 1, true);

    // rot_r.set_r_constants(7, 0, 45);
    // rot_r.set_rotation_pid(90, 120, 3);

    // left_front_wing.set_value(true);
    // right_front_wing.set_value(true);
    // intake(-12000);

	// mov_t.set_t_constants(5, 0, 45, 600);
	// mov_t.set_translation_pid(48, 127, 1, true);


}

void moveToUnderGoalWorlds(){
	double end_point_tolerance = 10;
    std::vector<CurvePoint> Path;
	bool reverse = true;

	double end_pose_x = 11; double end_pose_y = -20;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 20, 5, 1);
    CurvePoint newPoint0(4, 0, 1, 2, 40, 5, 1);
    CurvePoint newPoint1(6, -10, 1, 2, 40, 5, 1);
    CurvePoint newPoint2(11, -20, 1, 2, 40, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint0); Path.push_back(newPoint1); Path.push_back(newPoint2); 
	bool downOnce = false;

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			mtp.set_mtp_constants(6, 0, 150, 0, 70, 40);
			mtp.move_to_point(end_pose_x, end_pose_y, reverse, false, 0.5);
			utility::motor_deactivation();
			break;
		}
		FollowCurve(Path, 10, 10, 90, reverse);
		pros::delay(10);
	}
}

void Eclipse::Paths::rush_six_ball() {

    intake(-12000);
    odom_piston.set_value(true);

	mtp.set_mtp_constants(7, 45, 7, 35, 120, 90);
	mtp.move_to_point(-35, -10, true, false, 0.8);

    intake(12000);

    right_front_wing.set_value(true);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(115, 120, 0.7);

    right_front_wing.set_value(false);
    intake(12000);

	mov_t.set_t_constants(7, 0, 45, 600);
	mov_t.set_translation_pid(17, 120, 0.5, false);

	mtp.set_mtp_constants(7, 45, 7, 35, 120, 90);
	mtp.move_to_point(3, 7, true, false, 1.6);

    intake(-12000);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(-135, 90, 0.5);

    intake(12000);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(90, 90, 0.9);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(45, 120, 1, true);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(-40, 120, 1.2, false);

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(45, 120, 0.18, 0.4, true);

    left_wing.set_value(true);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(-25, 120, 0.4, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(15, 120, 0.6);

    left_wing.set_value(false);
    right_wing.set_value(false);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(-26, 120, 0.6, false);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(11, 120, 0.6, false);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(-165, 120, 0.9);

    intake(-12000);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(32, 120, 0.6, false);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(-18, 120, 1, false);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(115, 120, 1.3);

    intake(12000);

	mov_t.set_t_constants(12, 0, 45, 600);
	mov_t.set_translation_pid(65, 120, 1.5, false);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(180, 120, 0.7);

    cur_c.set_c_constants(20, 0, 45);
    cur_c.set_curve_pid(-90, 120, 0.1, 1.6, false);

    intake(-12000);

    left_front_wing.set_value(true);
    right_front_wing.set_value(true);

	mov_t.set_t_constants(16, 0, 45, 600);
	mov_t.set_translation_pid(50, 120, 0.8, false);

    left_front_wing.set_value(false);
    right_front_wing.set_value(false);

	mov_t.set_t_constants(12, 0, 45, 600);
	mov_t.set_translation_pid(-20, 120, 1, false);

}


void Eclipse::Paths::rush_disruption_close_side() {
    odom_piston.set_value(false);

    rot_r.set_r_constants(5, 0, 45);
    rot_r.set_rotation_pid(19, 90, 2);

    intake(12000); // intake

	mov_t.set_t_constants(8, 0, 45, 600);
	mov_t.set_translation_pid(47, 127, 3, false);

    left_front_wing.set_value(true);

    rot_r.set_r_constants(5, 0, 45);
    rot_r.set_rotation_pid(90, 90, 2);

	mov_t.set_t_constants(8, 0, 45, 600);
	mov_t.set_translation_pid(30, 127, 3, false);

	mov_t.set_t_constants(8, 0, 45, 600);
	mov_t.set_translation_pid(-30, 127, 3, false);

    left_front_wing.set_value(false);

    rot_r.set_r_constants(5, 0, 45);
    rot_r.set_rotation_pid(-50, 90, 2);

    intake(-12000); 
    pros::delay(500);

    rot_r.set_r_constants(5, 0, 45);
    rot_r.set_rotation_pid(0, 90, 2);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(-17, 110, 3, false);

    cur_c.set_c_constants(7, 0, 45);
    cur_c.set_curve_pid(90, 90, 0.1, 1, true);

	mov_t.set_t_constants(6, 0, 45, 300);
	mov_t.set_translation_pid(-38, 90, 1, false);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(0, 90, 3);

    cur_c.set_c_constants(7, 0, 45);
    cur_c.set_curve_pid(-45, 110, 0.22, 2, true);

    left_wing.set_value(true);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(-20, 90, 1.5, false);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(-90, 90, 3);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(-45, 90, 3);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(10, 90, 3, false);

    left_wing.set_value(false);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(-22, 90, 3, false);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(0, 90, 3);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(90, 90, 3);

    intake_motor.move_voltage(12000);
    intake_motor_secondary.move_voltage(-12000);
    right_front_wing.set_value(true);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(60, 40, 3, true);

    right_front_wing.set_value(false);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(-22, 90, 3, false);
}