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
	mov_t.set_translation_pid(-5, 90, -1, false);

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
	mov_t.set_translation_pid(26, 90, -1, false);

	intake_motor.move_voltage(0);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(-15, 90, -1, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(19, 90, -1, false);


    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(-10, 90, -1, false);


    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-132, 90);

	intake_motor.move_voltage(-12000);

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(25, 90, -1, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-160, 90);

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(-18, 90, -1, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

	intake_motor.move_voltage(12000);
	pros::delay(500);

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(24, 90, -1, false);

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(-10, 90, -1, false);

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

void moveArmDown() {
	while (true) {
		flywheel_arm.move_voltage(-12000);
         if (cata_sensor.get_value() == 1) {
            flywheel_arm.move_voltage(0);  // once arm is down, stop motor
			break;
         }
		 pros::delay(10);
	}
}

void skills() {

	odom_piston.set_value(false);
	//right_front_wing.set_value(true);
	left_wing.set_value(true);
	pros::delay(500);
	//right_front_wing.set_value(false);
	left_wing.set_value(false);
	intake_motor.move_voltage(-12000);


    cata_motor.move_voltage(12000);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(15, 110, -1, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-69, 110);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-18, 70, -1, false);

	left_wing.set_value(true);

	pros::delay(30000);

	// shoot

	left_wing.set_value(false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-135, 110);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(15, 110, -1, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-90, 110);

    cata_motor.move_voltage(0);

	traverse_to_other_side();

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(0, 110);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(12, 110, -1, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-9, 110, -1, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(8, 110, -1, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-9, 110, -1, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-90, 90);

	curve_to_mid_first();

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

	left_wing.set_value(true);
	right_wing.set_value(true);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-25, 110, -1, false);

	rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(30, 110, -1, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-30, 110, -1, false);

	rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(22, 110, -1, false);

	left_wing.set_value(false);
	right_wing.set_value(false);

	rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-28, 90);

	rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(0, 90);

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(-90, 90, 0.48, 4, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(10, 110, -1, false);

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(-180, 90, 0.37, 4, false);

	rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-180, 90);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-8, 110, -1, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(12, 110, -1, false);

	mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-5, 110, -1, false);

	rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

	cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(180, 110, 0.45, 4, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(24, 110, -1, false);

	rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

	left_wing.set_value(true);
	right_wing.set_value(true);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-38, 110, -1, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(24, 110, -1, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-48, 110, -1, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(24, 110, -1, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-48, 110, -1, false);


}

void new_skills() {

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(25, 90, 0.1, 4, true);

	mov_t.set_t_constants(5, 0, 35, 200);
	mov_t.set_translation_pid(8, 90, -1, false);

	cata_motor.move_voltage(-12000); cata_motor_secondary.move_voltage(-12000);
	pros::delay(50000);
	cata_motor.move_voltage(0); cata_motor_secondary.move_voltage(0);
}

void move_to_mid(){
	double end_point_tolerance = 15;
    std::vector<CurvePoint> Path;
	bool reverse = false;

	double end_pose_x = 50; double end_pose_y = 32;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 10, 5, 1);
    CurvePoint newPoint1(10, 0, 1, 2, 10, 5, 1);
    CurvePoint newPoint3(end_pose_x, end_pose_y, 2, 1, 10, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint1); Path.push_back(newPoint3);

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			mtp.set_mtp_constants(5, 35, 40, 0, 110, 110);
			mtp.move_to_point(end_pose_x, end_pose_y, reverse, false, 3000);
			utility::motor_deactivation();
			break;
		}
		FollowCurve(Path, 15, 5, 110, reverse);
		pros::delay(10);
	}
}

void go_to_lower_ball(){
	double end_point_tolerance = 22;
    std::vector<CurvePoint> Path;
	bool reverse = false;

	double end_pose_x = 40; double end_pose_y = 17;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 10, 5, 1);
    CurvePoint newPoint1(45, 0, 1, 2, 10, 5, 1);
    CurvePoint newPoint3(end_pose_x, end_pose_y, 2, 1, 10, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint1); Path.push_back(newPoint3);

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			mtp.set_mtp_constants(7, 35, 40, 0, 110, 110);
			mtp.move_to_point(end_pose_x, end_pose_y, reverse, false, 3000);
			utility::motor_deactivation();
			break;
		}
		FollowCurve(Path, 25, 6, 90, reverse);
		pros::delay(10);
	}
}

void move_to_pole(){
	double end_point_tolerance = 22;
    std::vector<CurvePoint> Path;
	bool reverse = false;

	double end_pose_x = 0; double end_pose_y = 2;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 20, 5, 1);
    CurvePoint newPoint1(2, -20, 1, 2, 40, 5, 1);
    CurvePoint newPoint3(end_pose_x, end_pose_y, 2, 1, 20, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint1); Path.push_back(newPoint3);

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			mtp.set_mtp_constants(7, 0, 40, 0, 110, 110);
			mtp.move_to_point(end_pose_x, end_pose_y, reverse, false, 3000);
			utility::motor_deactivation();
			break;
		}
		FollowCurve(Path, 25, 6, 90, reverse);
		pros::delay(10);
	}
}


void test6Ball(){
	intake_motor.move_voltage(12000);
	move_to_mid();

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 110);

	intake_motor.move_voltage(-12000);

	mtp.set_mtp_constants(6, 0, 150, 0, 110, 110);
	mtp.move_to_point(65, -18, false, false, 3000);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-160, 110);

	intake_motor.move_voltage(12000);

	go_to_lower_ball();

	mtp.set_mtp_constants(6, 0, 250, 0, 110, 110);
	mtp.move_to_point(60, 0, true, false, 3000);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 110);

	intake_motor.move_voltage(-12000);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(11, 110, -1, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-10, 110, -1, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(160, 110);

	intake_motor.move_voltage(12000);

	move_to_pole();

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-90, 110);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(10, 110, -1, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-90, 110);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-42, 110, -1, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-135, 110);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-20, 110, -1, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-180, 110);

   	mov_t.set_t_constants(7, 0, 35, 500);
	mov_t.set_translation_pid(-10, 127, -1, false);

  	mov_t.set_t_constants(7, 0, 35, 500);
	mov_t.set_translation_pid(8, 127, -1, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-90, 110);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(0, 110);

	intake_motor.move_voltage(-12000);

   	mov_t.set_t_constants(7, 0, 35, 500);
	mov_t.set_translation_pid(20, 127, -1, false);

   	mov_t.set_t_constants(7, 0, 35, 500);
	mov_t.set_translation_pid(-10, 127, -1, false);
}

void moveToBottomOfGoal(){
	double end_point_tolerance = 22;
    std::vector<CurvePoint> Path;
	bool reverse = true;

	double end_pose_x = -36; double end_pose_y = -8;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 20, 5, 1);
    CurvePoint newPoint1(-20, 0, 1, 2, 40, 5, 1);
    CurvePoint newPoint3(end_pose_x, end_pose_y, 2, 1, 20, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint1); Path.push_back(newPoint3);

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			mtp.set_mtp_constants(7, 0, 40, 0, 90, 50);
			mtp.move_to_point(end_pose_x, end_pose_y, reverse, false, 3000);
			utility::motor_deactivation();
			break;
		}
		FollowCurve(Path, 19, 6, 90, reverse);
		pros::delay(10);
	}
}

void moveToLowerTriball(){
	double end_point_tolerance = 22;
    std::vector<CurvePoint> Path;
	bool reverse = true;

	double end_pose_x = -8; double end_pose_y = -30;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 20, 5, 1);
    CurvePoint newPoint2(-18, -28, 1, 2, 40, 5, 1);
    CurvePoint newPoint3(end_pose_x, end_pose_y, 2, 1, 20, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint2); Path.push_back(newPoint3);

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

void safe_6ball() {

	odom_piston.set_value(true);
	pros::delay(500);

	intake_motor.move_voltage(12000);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(6, 110, -1, false);

	moveToBottomOfGoal();

	left_wing.set_value(true);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-15, 110, -1, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(10, 110, -1, false);

	left_wing.set_value(false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-10, 110, -1, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-90, 110);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-15, 110, -1, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(10, 110, -1, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(0, 110);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 110);

	intake_motor.move_voltage(-12000);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(12, 110, -1, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-14, 110, -1, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(180, 110);

	intake_motor.move_voltage(12000);

	moveToLowerTriball();

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(0, 110);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(12, 110, -1, false);

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(-40, 110, 0.2, 4, true);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(125, 110);

	intake_motor.move_voltage(-12000);
	pros::delay(500);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(45, 110);

	intake_motor.move_voltage(12000);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(20, 80, -1, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(180, 70);

	front_wings.set_value(true);
	pros::delay(500);

	intake_motor.move_voltage(-12000);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(60, 110, -1, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-15, 110, -1, false);


}


void knock_mid_goals(){
	double end_point_tolerance = 15;
    std::vector<CurvePoint> Path;
	bool reverse = false;

	double end_pose_x = 46; double end_pose_y = -22;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 20, 5, 1);
    CurvePoint newPoint1(10, 0, 1, 2, 40, 5, 1);
    CurvePoint newPoint3(end_pose_x, end_pose_y, 2, 1, 20, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint1); Path.push_back(newPoint3);

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			mtp.set_mtp_constants(7, 0, 300, 0, 110, 110);
			mtp.move_to_point(end_pose_x, end_pose_y, reverse, false, 3000);
			utility::motor_deactivation();
			break;
		}
		FollowCurve(Path, 15, 9, 110, reverse);
		pros::delay(10);
	}
}

void go_to_team_zone(){
	double end_point_tolerance = 15;
    std::vector<CurvePoint> Path;
	bool reverse = true;

	double end_pose_x = 10; double end_pose_y = 28;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 20, 5, 1);
    CurvePoint newPoint0(45, 0, 1, 2, 40, 5, 1);
    CurvePoint newPoint1(10, 6, 1, 2, 40, 5, 1);
    CurvePoint newPoint3(end_pose_x, end_pose_y, 2, 1, 20, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint0); Path.push_back(newPoint1); Path.push_back(newPoint3);

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			mtp.set_mtp_constants(7, 0, 300, 0, 110, 110);
			mtp.move_to_point(end_pose_x, end_pose_y, reverse, false, 3000);
			utility::motor_deactivation();
			break;
		}
		FollowCurve(Path, 10, 6, 70, reverse);
		pros::delay(10);
	}
}


void close_side() {

	odom_piston.set_value(true);
	pros::delay(500);

	front_wings.set_value(true);
	pros::delay(500);
	front_wings.set_value(false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(110, 40);

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(180, 90, 0.6, 4, true);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-40, 90, -1, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(6, 90, -1, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 70);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(40, 90, -1, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(0, 70);

	front_wings.set_value(true);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(26, 90, -1, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-8, 90, -1, false);


	// knock_mid_goals();

    // rot_r.set_r_constants(6, 0, 45);
    // rot_r.set_rotation_pid(0, 40);

	// front_wings.set_value(true);
	// pros::delay(500);
	// front_wings.set_value(false);

	// mtp.set_mtp_constants(6, 0, 150, 0, 110, 110);
	// mtp.move_to_point(45, 20, false, true);

	// go_to_team_zone();

    // rot_r.set_r_constants(6, 0, 45);
    // rot_r.set_rotation_pid(-45, 70);

    // cur_c.set_c_constants(6, 0, 45);
    // cur_c.set_curve_pid(-90, 90, 0.56, true);

    // mov_t.set_t_constants(5, 0, 35, 500);
	// mov_t.set_translation_pid(-27, 90, false);

    // mov_t.set_t_constants(5, 0, 35, 500);
	// mov_t.set_translation_pid(5, 90, false);

}

void left_no_odom() {

	odom_piston.set_value(true);
	intake_motor.move_voltage(-12000);
	front_wings.set_value(true);
	flywheel_arm.move_voltage(-12000);
	pros::delay(300);
	front_wings.set_value(false);
	flywheel_arm.move_voltage(0);
	pros::delay(200);

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(20, 110, 0.6, 4, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(33, 90, -1, false);

	front_wings.set_value(true);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 110);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(20, 110, -1, true);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-10, 110, -1, true);

	front_wings.set_value(false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-170, 110);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(27, 90, -1, true);

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(-90, 90, 0.2, 4, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(10, 110, -1, false);

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(-45, 90, 0.2, 4, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-45, 60);

	left_wing.set_value(true);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-24, 110, -1, true);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(5, 60, -1, true);

	left_wing.set_value(false);	

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(-90, 90, 0.1, 4, true);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-90, 60);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-30, 60, -1, true);
}

void debug_pp(){
	double end_point_tolerance = 18;
    std::vector<CurvePoint> Path;
	bool reverse = false;

	double end_pose_x = 100; double end_pose_y = 81;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 20, 5, 1);
    CurvePoint newPoint0(18, -20, 1, 2, 40, 5, 1);
    CurvePoint newPoint1(102, -23, 2, 1, 20, 5, 1);
    CurvePoint newPoint2(90, 15, 2, 1, 20, 5, 1);
    CurvePoint newPoint3(71, 45, 2, 1, 20, 5, 1);
    CurvePoint newPoint4(end_pose_x, end_pose_y, 2, 1, 20, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint0); Path.push_back(newPoint1); Path.push_back(newPoint2); Path.push_back(newPoint3); Path.push_back(newPoint4); 

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
		mtp.set_mtp_constants(6, 0, 150, 0, 70, 90);
		mtp.move_to_point(end_pose_x, end_pose_y, false, false, 2);
			utility::motor_deactivation();
			break;
		}
		FollowCurve(Path, 25, 6, 70, reverse);
		pros::delay(10);
	}
}


void test_path_debug() {
	pros::delay(500);
	debug_pp();
}


void moveToUnderGoal(){
	double end_point_tolerance = 10;
    std::vector<CurvePoint> Path;
	bool reverse = true;

	double end_pose_x = 31; double end_pose_y = -26;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 20, 5, 1);
    CurvePoint newPoint0(0, 0, 1, 2, 40, 5, 1);
    CurvePoint newPoint1(9, -15, 1, 2, 40, 5, 1);
    CurvePoint newPoint4(end_pose_x, end_pose_y, 2, 1, 20, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint0); Path.push_back(newPoint1);  Path.push_back(newPoint4); 

	bool downOnce = false;

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(9 - utility::get_x(), 2) + pow(-15 - utility::get_y(), 2))) <= 12 && downOnce == false){
			left_wing.set_value(true);
			pros::delay(200);
			left_wing.set_value(false);
			downOnce = true;
		}
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			mtp.set_mtp_constants(6, 0, 150, 0, 70, 40);
			mtp.move_to_point(end_pose_x, end_pose_y, reverse, false, 0.5);
			utility::motor_deactivation();
			break;
		}
		FollowCurve(Path, 20, 6, 90, reverse);
		pros::delay(10);
	}
}


void new6ballmecha() {

	odom_piston.set_value(true);
	intake_motor.move_voltage(12000);
	front_wings.set_value(true);
	flywheel_arm.move_voltage(-12000);
	pros::delay(300);
	front_wings.set_value(false);
	flywheel_arm.move_voltage(0);
	pros::delay(200);

	rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-88, 110);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(30, 90, -1, true);

	moveToUnderGoal();

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-180, 110);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-5, 127, -1, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(6, 127, -1, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-90, 110);

    rot_r.set_r_constants(6, 0, 45);

    rot_r.set_rotation_pid(-0, 110);

	intake_motor.move_voltage(-12000);

	mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(14, 127, -1, false);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-13, 110, -1, false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-55, 110);

	intake_motor.move_voltage(12000);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(35, 110, -1, true);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-90, 110);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(12, 90, -1, true);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-5, 110, -1, true);

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(-100, 100, 0.2, 4, true);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(60, 90);

	intake_motor.move_voltage(-12000);
	pros::delay(200);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-38, 90);

	intake_motor.move_voltage(12000);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(26, 110, -1, true);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

	intake_motor.move_voltage(-12000);

	front_wings.set_value(true);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(35, 110, -1, false);

	front_wings.set_value(false);

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(0, 100, 0.2, 4, true);

	// mtp.set_mtp_constants(7, 0, 200, 0, 90, 0);
	// mtp.move_to_point(0, 35, false, true, 3000);

}

void driveBack() {
	odom_piston.set_value(true);
	flywheel_arm.move_voltage(-12000);
	pros::delay(300);
	flywheel_arm.move_voltage(0);
	pros::delay(200);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-50, 110, -1, false);

	pros::delay(1000);

    mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(24, 110, -1, true);

}

void moveToMid(){
	double end_point_tolerance = 10;
    std::vector<CurvePoint> Path;
	bool reverse = false;

	double end_pose_x = 62; double end_pose_y = -15;

    CurvePoint StartPos(utility::get_x(), utility::get_y(), 4, 2, 20, 5, 1);
    CurvePoint newPoint0(14, -6, 1, 2, 40, 5, 1);
    CurvePoint newPoint4(end_pose_x, end_pose_y, 2, 1, 20, 5, 1);
    Path.push_back(StartPos); Path.push_back(newPoint0);  Path.push_back(newPoint4); 

    while (true){ 
		odom.update_odom();
		if(fabs(sqrt(pow(end_pose_x - utility::get_x(), 2) + pow(end_pose_y - utility::get_y(), 2))) <= fabs(end_point_tolerance)){
			// mtp.set_mtp_constants(6, 0, 150, 0, 70, 0);
			// mtp.move_to_point(end_pose_x, end_pose_y, reverse, false, 0);
			utility::motor_deactivation();
			break;
		}
		FollowCurve(Path, 10, 6, 90, reverse);
		pros::delay(10);
	}
}

void moveArmDownMecha() {
	while (true) {
		flywheel_arm.move_voltage(-12000);
         if (cata_sensor.get_value() == 1) {
            flywheel_arm.move_voltage(0);  // once arm is down, stop motor
			break;
         }
		 pros::delay(10);
	}
}

void closeSideMecha(){
	odom_piston.set_value(true);
	intake_motor.move_voltage(12000);
	front_wings.set_value(true);
	flywheel_arm.move_voltage(-12000);
	pros::delay(1000);
	front_wings.set_value(false);
	flywheel_arm.move_voltage(0);
	pros::delay(200);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(15, 90);

	mtp.set_mtp_constants(6, 0, 250, 0, 90, 40);
	mtp.move_to_point(51, -12, false, false, 2);
	pros::delay(500);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(10, 90);

	mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-36, 90, -1, true);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(180, 90);

	intake_motor.move_voltage(-12000);
	pros::delay(700);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-90, 90);

	mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(8, 70, -1, true);

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(-45, 100, 0.2, 4, false);

	left_wing.set_value(true);

	mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-15, 70, -1, true);

	mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(6, 70, -1, true);

	left_wing.set_value(false);

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(-90, 90, 0.36, 4, true);

	// move arm down

	// mov_t.set_t_constants(5, 0, 35, 500);
	// mov_t.set_translation_pid(-29.5, 70, true); 
}

void testcloseSideMecha(){
	odom_piston.set_value(true);
	intake_motor.move_voltage(12000);
	flywheel_arm.move_voltage(-12000);
	pros::delay(1000);
	flywheel_arm.move_voltage(0);
	pros::delay(200);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(20, 90);

	mtp.set_mtp_constants(6, 0, 250, 0, 90, 40);
	mtp.move_to_point(51, -12, false, false, 2);
	pros::delay(500);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(90, 90);

	front_wings.set_value(true);

	mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(20, 90, -1, true);

	mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-10, 90, -1, true);

	front_wings.set_value(false);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-170, 90);

	mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(36, 90, -1, true);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(140, 90);

	intake_motor.move_voltage(-12000);
	pros::delay(700);

    rot_r.set_r_constants(6, 0, 45);
    rot_r.set_rotation_pid(-90, 90);

	mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(8, 70, -1, true);

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(-45, 100, 0.16, 4, false);

	left_wing.set_value(true);

	mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(-15, 70, -1, true);

	mov_t.set_t_constants(5, 0, 35, 500);
	mov_t.set_translation_pid(6, 70, -1, true);

	left_wing.set_value(false);

    cur_c.set_c_constants(6, 0, 45);
    cur_c.set_curve_pid(-110, 90, 0.2, 4, true);

	moveArmDownMecha();
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