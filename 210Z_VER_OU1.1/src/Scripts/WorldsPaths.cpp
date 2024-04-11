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

    intake_motor.move_voltage(-12000);
    intake_motor_secondary.move_voltage(12000);

    cur_c.set_c_constants(8, 0, 45);
    cur_c.set_curve_pid(25, 110, 0.3, 0.35, false);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(48, 127, 3, true);

    pros::delay(200);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(-20, 110, 3, true);

    cur_c.set_c_constants(7, 0, 45);
    cur_c.set_curve_pid(90, 90, 0.1, 1, true);

	mov_t.set_t_constants(6, 0, 45, 300);
	mov_t.set_translation_pid(-38, 50, 3, true);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(0, 60, 3);

    cur_c.set_c_constants(7, 0, 45);
    cur_c.set_curve_pid(-45, 110, 0.18, 2, true);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(-23, 70, 3, true);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(10, 70, 3, true);

    cur_c.set_c_constants(7, 0, 45);
    cur_c.set_curve_pid(-90, 110, 0.21, 2, true);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(-90, 40, 3);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(0, 40, 3);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(90, 40, 3);

    intake_motor.move_voltage(12000);
    intake_motor_secondary.move_voltage(-12000);

	mov_t.set_t_constants(6, 0, 45, 600);
	mov_t.set_translation_pid(40, 50, 3, true);
}

void Eclipse::Paths::local_six_ball() {

    intake(12000);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(11, 110, 3, true);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(-31, 110, 3, true);

    cur_c.set_c_constants(7, 0, 45);
    cur_c.set_curve_pid(-45, 110, 0.18, 2, true);

    left_wing.set_value(true);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(-22, 110, 3, true);

    left_wing.set_value(false);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(-90, 110, 1);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(-22, 110, 1, true);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(8, 110, 1, true);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(110, 110, 1);

    intake(-12000);

	mov_t.set_t_constants(8, 0, 45, 600);
	mov_t.set_translation_pid(24, 120, 0.7, true);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(-13, 110, 0.7, true);

    intake(12000);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(34, 110, 1);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(40, 110, 4, true);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(0, 110, 1);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(17, 90, 0.5, true);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-12, 110, 2, true);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(150, 110, 3);

    intake(-12000);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(47, 110, 3);

    intake(12000);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(32, 127, 1, true);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(180, 110, 3);

    left_front_wing.set_value(true);
    right_front_wing.set_value(true);
    intake(-12000);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(48, 127, 1, true);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(180, 110, 0.5);

    left_front_wing.set_value(false);
    right_front_wing.set_value(false);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(-48, 127, 1, true);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(100, 110, 1);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(-40, 127, 1, true);


}

void Eclipse::Paths::global_close_side() {

}
void Eclipse::Paths::global_six_ball() {

}

void moveToUnderGoalWorlds(){
	double end_point_tolerance = 10;
    std::vector<CurvePoint> Path;
	bool reverse = true;

	double end_pose_x = 10; double end_pose_y = -22;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 20, 5, 1);
    CurvePoint newPoint0(-3, 0, 1, 2, 40, 5, 1);
    CurvePoint newPoint1(5, -17, 1, 2, 40, 5, 1);
    CurvePoint newPoint2(10, -22, 1, 2, 40, 5, 1);
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

    intake(12000);

	mtp.set_mtp_constants(7, 45, 7, 35, 110, 90);
	mtp.move_to_point(39, 14, false, false, 1.3);

	mtp.set_mtp_constants(7, 45, 5, 35, 110, 90);
	mtp.move_to_point(-3, 0, true, false, 1.1);

    intake(-12000);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(45, 110, 0.3);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(-90, 110, 0.4);

    intake(12000);

	mtp.set_mtp_constants(7, 45, 5, 35, 110, 90);
	mtp.move_to_point(0, 24, false, false, 2);

    moveToUnderGoalWorlds();

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(-180, 110, 3);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(-22, 110, 0.7, true);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(8, 110, 1, true);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(20, 110, 3);

    intake(-12000);

	mov_t.set_t_constants(8, 0, 45, 600);
	mov_t.set_translation_pid(50, 120, 0.7, true);

	mov_t.set_t_constants(5, 0, 45, 600);
	mov_t.set_translation_pid(-11, 110, 0.7, true);

    intake(12000);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(70, 110, 0.5);

	mtp.set_mtp_constants(7, 45, 5, 35, 110, 90);
	mtp.move_to_point(26, 24, false, false, 1.3);

	mtp.set_mtp_constants(7, 45, 5, 35, 110, 90);
	mtp.move_to_point(34, 0, false, false, 0.5);

    intake(-12000);

	mtp.set_mtp_constants(7, 45, 5, 35, 110, 90);
	mtp.move_to_point(34, -24, false, false, 1.2);

	mtp.set_mtp_constants(5, 45, 8, 35, 110, 90);
	mtp.move_to_point(53, 24, false, false, 1.3);

    intake(12000);

    rot_r.set_r_constants(7, 0, 45);
    rot_r.set_rotation_pid(90, 110, 1);

    intake(-12000);

	mov_t.set_t_constants(8, 0, 45, 600);
	mov_t.set_translation_pid(50, 120, 1, true);

    cur_c.set_c_constants(8, 0, 45);
    cur_c.set_curve_pid(0, 110, 0.1, 2, true);

}
void Eclipse::Paths::rush_disruption_close_side() {

}