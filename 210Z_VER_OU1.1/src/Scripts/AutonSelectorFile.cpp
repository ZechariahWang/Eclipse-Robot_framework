#include "Config/Globals.hpp"
#include "main.h"

using namespace Eclipse;

void auton1(){}
void auton2(){}
void auton3(){}
void auton4(){}
void auton5(){}
void auton6(){}
void auton7(){}
void auton8(){}
void auton9(){}
void auton10(){}

void PurePursuitTestPath(){
	double end_point_tolerance = 15;
    std::vector<CurvePoint> Path;
	bool reverse = false;

	double end_pose_x = 56; double end_pose_y = 24;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 10, 5, 1);
    CurvePoint newPoint1(20, 8, 1, 2, 10, 5, 1);
    CurvePoint newPoint2(42, 17, 2, 1, 10, 5, 1);
    CurvePoint newPoint3(end_pose_x, end_pose_y, 2, 1, 10, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint1); Path.push_back(newPoint2); Path.push_back(newPoint3);

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			mtp.set_mtp_constants(3, 35, 40, 0, 40, 40);
			mtp.move_to_point(end_pose_x, end_pose_y, reverse, false, 3000);
			utility::motor_deactivation();
			break;
		}
		FollowCurve(Path, 0, 5, 2, reverse);
		pros::delay(10);
	}
}

void ReversePurePursuitTestPath(){
	double end_point_tolerance = 15;
    std::vector<CurvePoint> Path;
	bool reverse = true;

	double end_pose_x = 12; double end_pose_y = 52;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 10, 5, 1);
    CurvePoint newPoint1(31, 12, 1, 2, 10, 5, 1);
    CurvePoint newPoint2(26, 8, 2, 1, 10, 5, 1);
    CurvePoint newPoint3(end_pose_x, end_pose_y, 2, 1, 10, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint1); Path.push_back(newPoint2); Path.push_back(newPoint3);

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			mtp.set_mtp_constants(3, 35, 40, 0, 40, 40);
			mtp.move_to_point(end_pose_x, end_pose_y, reverse, false, 3000);
			utility::motor_deactivation();
			break;
		}
		FollowCurve(Path, 0, 5, 2, reverse);
		pros::delay(10);
	}
}


// main driver function for the left side auton route
void left_side() {
	std::cout << "running"  << std::endl;
	blocker.set_value(true);

	//move_to_goal_left_side(); // move to goal 

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(-5, 90, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-20, 90);

	//move_to_elevation_pole(); // move to elevation pole
}

// main driver function for the right side auton route
void right_side() {
	blocker.set_value(true);
	pros::delay(500);
	intake_motor.move_voltage(-12000);

	PurePursuitTestPath();

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 40);

	intake_motor.move_voltage(12000);

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(26, 90, false);

	intake_motor.move_voltage(0);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(-15, 90, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(19, 90, false);


    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(-10, 90, false);


    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-132, 90);

	intake_motor.move_voltage(-12000);

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(25, 90, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-160, 90);

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(-18, 90, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

	intake_motor.move_voltage(12000);
	pros::delay(500);

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(24, 90, false);

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(-10, 90, false);

    // rot_r.set_r_constants(6, 0, 45);
    // rot_r.set_rotation_pid(90, 90);

	// mov_t.set_t_constants(5, 0, 35, 200);
	// mov_t.set_translation_pid(10, 60, false);

	// mov_t.set_t_constants(5, 0, 35, 200);
	// mov_t.set_translation_pid(-19, 90, false);

    // rot_r.set_r_constants(6, 0, 45);
    // rot_r.set_rotation_pid(90, 90);

	// mov_t.set_t_constants(5, 0, 35, 200);
	// mov_t.set_translation_pid(35, 90, false);

}

void traverse_to_other_side(){
	double end_point_tolerance = 22;
    std::vector<CurvePoint> Path;
	bool reverse = false;

	double end_pose_x = 10; double end_pose_y = 105;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 20, 5, 1);
    CurvePoint newPoint2(-10, 80, 2, 1, 20, 5, 1);
    CurvePoint newPoint3(10, 110, 2, 1, 20, 5, 1);
    CurvePoint newPoint4(end_pose_x, end_pose_y, 2, 1, 20, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint2); Path.push_back(newPoint3); Path.push_back(newPoint4);

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			mtp.set_mtp_constants(7, 0, 40, 0, 70, 50);
			mtp.move_to_point(end_pose_x, end_pose_y, reverse, false, 3000);
			utility::motor_deactivation();
			break;
		}
		FollowCurve(Path, 25, 6, 90, reverse);
		pros::delay(10);
	}
}

void curve_to_mid_first(){
	double end_point_tolerance = 22;
    std::vector<CurvePoint> Path;
	bool reverse = true;

	double end_pose_x = 55; double end_pose_y = 65;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 20, 5, 1);
    CurvePoint newPoint1(23, 65, 1, 2, 40, 5, 1);
    CurvePoint newPoint2(55, 65, 1, 2, 40, 5, 1);
    CurvePoint newPoint4(end_pose_x, end_pose_y, 2, 1, 20, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint1);  Path.push_back(newPoint2); Path.push_back(newPoint4);

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			mtp.set_mtp_constants(7, 0, 40, 0, 90, 50);
			mtp.move_to_point(end_pose_x, end_pose_y, reverse, false, 3000);
			utility::motor_deactivation();
			break;
		}
		FollowCurve(Path, 25, 6, 90, reverse);
		pros::delay(10);
	}
}

void curve_to_other_side_of_goal(){
	double end_point_tolerance = 22;
    std::vector<CurvePoint> Path;
	bool reverse = false;

	double end_pose_x = 100; double end_pose_y = 105;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 20, 5, 1);
    CurvePoint newPoint1(65, 60, 1, 2, 40, 5, 1);
    CurvePoint newPoint2(110, 80, 1, 2, 40, 5, 1);
    CurvePoint newPoint4(end_pose_x, end_pose_y, 2, 1, 20, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint1);  Path.push_back(newPoint2); Path.push_back(newPoint4);

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			mtp.set_mtp_constants(7, 0, 40, 0, 90, 50);
			mtp.move_to_point(end_pose_x, end_pose_y, reverse, false, 3000);
			utility::motor_deactivation();
			break;
		}
		FollowCurve(Path, 25, 6, 50, reverse);
		pros::delay(10);
	}
}

void skills() {

    cata_motor.move_voltage(12000);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(15, 110, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-73, 110);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-17, 70, false);

	left_wing.set_value(true);

	//pros::delay(30000);

	// shoot

	left_wing.set_value(false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-135, 110);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(15, 110, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-90, 110);

    cata_motor.move_voltage(0);

	traverse_to_other_side();

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(0, 110);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(12, 110, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-9, 110, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(8, 110, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-9, 110, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-90, 90);

	curve_to_mid_first();

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

	left_wing.set_value(true);
	right_wing.set_value(true);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-25, 110, false);

	rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(30, 110, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-30, 110, false);

	rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(22, 110, false);

	left_wing.set_value(false);
	right_wing.set_value(false);

	rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-28, 90);

	rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(0, 90);

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(-90, 90, 0.48, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(6, 110, false);

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(-180, 90, 0.37, false);

	rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-180, 90);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-8, 110, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(12, 110, false);

	mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-5, 110, false);

	rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

	cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(180, 110, 0.45, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(24, 110, false);

	rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

	left_wing.set_value(true);
	right_wing.set_value(true);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-38, 110, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(24, 110, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-48, 110, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(24, 110, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-48, 110, false);


}

void new_skills() {

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(25, 90, 0.1, true);

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(8, 90, false);

	cata_motor.move_voltage(-12000); cata_motor_secondary.move_voltage(-12000);
	pros::delay(50000);
	cata_motor.move_voltage(0); cata_motor_secondary.move_voltage(0);
}

void AutonSelectorPrimary(const u_int16_t autonType){
    switch (autonType){
    case 0: auton1();                           break;
    case 1: auton2();                           break;
    case 2: auton3();                           break;
    case 3: auton4();                           break;
    case 4: auton5();                           break;
    case 5: auton6();                           break;
    case 6:                                     break;
    case 7:                                     break;
    case 8:                                     break;
    case 9:                                     break;
    case 10:                                    break;
    default:                                    break;
    }
}