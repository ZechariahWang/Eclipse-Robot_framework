#include "main.h"

using namespace Eclipse;

void move_to_goal_left_side(){
	double end_point_tolerance = 15;
    std::vector<CurvePoint> Path;
	bool reverse = false;

	double end_pose_x = 27; double end_pose_y = 17;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 10, 5, 1);
    CurvePoint newPoint1(10, 17, 1, 2, 10, 5, 1);
    CurvePoint newPoint3(end_pose_x, end_pose_y, 2, 1, 10, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint1); Path.push_back(newPoint3);

    int8_t overRideTimer = 0;
    int distanceThreshold = 1;
    int timer = 2000;
    double prev_x = 0;
    double prev_y = 0;

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			mtp.set_mtp_constants(3, 35, 40, 0, 40, 40);
			mtp.move_to_point(end_pose_x, end_pose_y, reverse, false);
			utility::motor_deactivation();
			break;
		}
        if (utility::get_x() - prev_x < 1 && utility::get_y() - prev_y < 1) { overRideTimer++; }
        if (overRideTimer > 2000.0) {
            utility::motor_deactivation();
			break;
        }
		FollowCurve(Path, 0, 5, 2, reverse);
		pros::delay(10);
		prev_x = utility::get_x();
		prev_y = utility::get_y();
	}
}

void move_to_elevation_pole(){
	double end_point_tolerance = 15;
    std::vector<CurvePoint> Path;
	bool reverse = true;

	double end_pose_x = 1; double end_pose_y = -37;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 10, 5, 1);
    CurvePoint newPoint1(10, 17, 1, 2, 10, 5, 1);
	CurvePoint newPoint2(3, 0, 1, 2, 10, 5, 1);
    CurvePoint newPoint3(end_pose_x, end_pose_y, 2, 1, 10, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint1); Path.push_back(newPoint2); Path.push_back(newPoint3);

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			mtp.set_mtp_constants(3, 35, 40, 0, 40, 40);
			mtp.move_to_point(end_pose_x, end_pose_y, reverse, false);
			utility::motor_deactivation();
			break;
		}
		FollowCurve(Path, 0, 5, 2, reverse);
		pros::delay(10);
	}
}

void PrimaryLeftSide() {

    blocker.set_value(true);
    pros::delay(500);

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(45, 90, 0.35, true);

	mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-10, 90, false);

	mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(10, 90, false);

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(0, 90, 0.35, false);

    climber.set_value(true);

    pros::delay(500);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(20, 90, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(142, 40);

    blocker.set_value(true);
    pros::delay(500);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(8, 90, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(135, 40);

	mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-36, 60, false);

    // move_to_elevation_pole();
}