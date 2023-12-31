#include "Config/Globals.hpp"
#include "main.h"

using namespace Eclipse;


void MoveToCenterTriball(){
	double end_point_tolerance = 15;
    std::vector<CurvePoint> Path;
	bool reverse = false;

	double end_pose_x = 55; double end_pose_y = 6;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 10, 5, 1);
    CurvePoint newPoint1(20, 6, 1, 2, 10, 5, 1);
    CurvePoint newPoint2(42, 6, 2, 1, 10, 5, 1);
    CurvePoint newPoint3(end_pose_x, end_pose_y, 2, 1, 10, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint1); Path.push_back(newPoint2); Path.push_back(newPoint3);

    double prev_x = 0;
    double prev_y = 0;
    int overRideTimer = 0;

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			mtp.set_mtp_constants(3, 35, 40, 0, 60, 60);
			mtp.move_to_point(end_pose_x, end_pose_y, reverse, false);
			utility::motor_deactivation();
			break;
		}
        if (fabs(utility::get_x() - prev_x) < 2 && fabs(utility::get_y() - prev_y) < 2) { overRideTimer++; }
        if (overRideTimer > 200.0) {
            utility::motor_deactivation();
            break;
        }  
		FollowCurve(Path, 0, 5, 2, reverse);
		pros::delay(10);
	}
}

void MoveToLeftTriball(){
	double end_point_tolerance = 15;
    std::vector<CurvePoint> Path;
	bool reverse = false;

	double end_pose_x = 45; double end_pose_y = 29;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 10, 5, 1);
    CurvePoint newPoint1(20, 8, 1, 2, 10, 5, 1);
    CurvePoint newPoint2(45, 8, 2, 1, 10, 5, 1);
    CurvePoint newPoint3(end_pose_x, end_pose_y, 2, 1, 10, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint1); Path.push_back(newPoint2); Path.push_back(newPoint3);

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			mtp.set_mtp_constants(3, 35, 40, 0, 60, 60);
			mtp.move_to_point(end_pose_x, end_pose_y, reverse, false);
			utility::motor_deactivation();
			break;
		}
		FollowCurve(Path, 0, 5, 2, reverse);
		pros::delay(10);
	}
}

void MoveToGoalFirstRun(){
	double end_point_tolerance = 15;
    std::vector<CurvePoint> Path;
	bool reverse = false;

	double end_pose_x = 50; double end_pose_y = -14;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 10, 5, 1);
    CurvePoint newPoint1(52, 0, 1, 2, 10, 5, 1);
    CurvePoint newPoint2(52, -10, 2, 1, 10, 5, 1);
    CurvePoint newPoint3(end_pose_x, end_pose_y, 2, 1, 10, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint1); Path.push_back(newPoint2); Path.push_back(newPoint3);

    double prev_x = 0;
    double prev_y = 0;
    int overRideTimer = 0;

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			mtp.set_mtp_constants(3, 35, 40, 0, 60, 60);
			mtp.move_to_point(end_pose_x, end_pose_y, reverse, false);
			utility::motor_deactivation();
			break;
		}
        if (fabs(utility::get_x() - prev_x) < 2 && fabs(utility::get_y() - prev_y) < 2) { overRideTimer++; }
        if (overRideTimer > 100.0) {
            utility::motor_deactivation();
            break;
        }  
		FollowCurve(Path, 0, 5, 2, reverse);
		pros::delay(10);
	}
}

void MoveToLowerTriball(){
	double end_point_tolerance = 15;
    std::vector<CurvePoint> Path;
	bool reverse = false;

	double end_pose_x = 29; double end_pose_y = 27;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 10, 5, 1);
    CurvePoint newPoint1(29, 0, 1, 2, 10, 5, 1);
    CurvePoint newPoint2(end_pose_x, end_pose_y, 2, 1, 10, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint1); Path.push_back(newPoint2); 

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			mtp.set_mtp_constants(3, 35, 40, 0, 60, 60);
			mtp.move_to_point(end_pose_x, end_pose_y, reverse, false);
			utility::motor_deactivation();
			break;
		}
		FollowCurve(Path, 0, 5, 2, reverse);
		pros::delay(10);
	}
}

void MoveToBottomOfGoal(){
	double end_point_tolerance = 15;
    std::vector<CurvePoint> Path;
	bool reverse = true;

	double end_pose_x = -3; double end_pose_y = 5;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 10, 5, 1);
    CurvePoint newPoint1(6, 0, 1, 2, 10, 5, 1);
    CurvePoint newPoint2(-3, -18, 1, 2, 10, 5, 1);
    CurvePoint newPoint3(end_pose_x, end_pose_y, 2, 1, 10, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint1); Path.push_back(newPoint2); Path.push_back(newPoint3); 

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			mtp.set_mtp_constants(3, 35, 40, 0, 60, 60);
			mtp.move_to_point(end_pose_x, end_pose_y, reverse, false);
			utility::motor_deactivation();
			break;
		}
		FollowCurve(Path, 0, 5, 2, reverse);
		pros::delay(10);
	}
}


void PrimaryRightSide() {
    blocker.set_value(true);
    wings.set_value(true);
	pros::delay(500);
    wings.set_value(false);
	intake_motor.move_voltage(-12000); // intake

    MoveToCenterTriball();

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(-8, 90, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(70, 90);

    intake_motor.move_voltage(12000);
	pros::delay(500);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(0, 90);

    intake_motor.move_voltage(-12000);

    MoveToLeftTriball();

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(-5, 90, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

    intake_motor.move_voltage(12000);

	// wings.set_value(true);

    MoveToGoalFirstRun();

    mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(-5, 90, false);

    mtp.set_mtp_constants(5, 35, 80, 0, 90, 90);
    mtp.TurnToPoint(5, 24);

    intake_motor.move_voltage(-12000);

    MoveToLowerTriball();

	rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-90, 90);

    MoveToBottomOfGoal();

	intake_motor.move_voltage(12000);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(225, 90);

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(180, 90, 0.55, true);


    // rot_r.set_r_constants(6, 0, 45);
    // rot_r.set_rotation_pid(35, 40);

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(-20, 90, false);
}