#include "Config/Globals.hpp"
#include "main.h"
#include <string>

/**
 * L2: Outtake
 * R2: Intake
 * L1: Left Wing
 * R2: Right Wing
 * DOWN: Both backwings
 * RIGHT: Move arm down
 * LEFT: Move arm up
 * UP: Flywheel backwards
 * Y: Climber
 * B: Flywheel forward
 * A: Front wings
 * X: NULL
 */


const uint64_t LINE_SENSOR_VAL = 4096; // max value to be considered an object
const uint64_t LINE_SENSOR_THRESHHOLD = 1000; // min value to notdetect anything

int32_t receive_and_validate_input_key(std::string key, bool hold) {
    pros::controller_digital_e_t digital_key;

    if (key == "R1")         digital_key = pros::E_CONTROLLER_DIGITAL_R1;
    else if (key == "R2")    digital_key = pros::E_CONTROLLER_DIGITAL_R2;
    else if (key == "L1")    digital_key = pros::E_CONTROLLER_DIGITAL_L1;
    else if (key == "L2")    digital_key = pros::E_CONTROLLER_DIGITAL_L2;
    else if (key == "UP")    digital_key = pros::E_CONTROLLER_DIGITAL_UP;
    else if (key == "LEFT")  digital_key = pros::E_CONTROLLER_DIGITAL_LEFT;
    else if (key == "DOWN")  digital_key = pros::E_CONTROLLER_DIGITAL_DOWN;
    else if (key == "RIGHT") digital_key = pros::E_CONTROLLER_DIGITAL_RIGHT;
    else if (key == "X")     digital_key = pros::E_CONTROLLER_DIGITAL_X;
    else if (key == "Y")     digital_key = pros::E_CONTROLLER_DIGITAL_Y;
    else if (key == "A")     digital_key = pros::E_CONTROLLER_DIGITAL_A;
    else if (key == "B")     digital_key = pros::E_CONTROLLER_DIGITAL_B;
    else return 0;

    return hold ? controller.get_digital(digital_key) : controller.get_digital_new_press(digital_key);
}


void power_intake(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){ intake_motor.move_voltage(12000); }
    else if ((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))){ intake_motor.move_voltage(-12000); } // intake
    else{ intake_motor.move_voltage(0); }
}

bool left_wing_extended = false;
bool right_wing_extended = false;
bool both_wings_extended = false;
void extend_wings() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        both_wings_extended = !both_wings_extended;
        left_wing_extended = both_wings_extended;
        right_wing_extended = both_wings_extended;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        left_wing_extended = !left_wing_extended;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        right_wing_extended = !right_wing_extended;
        std::cout << right_wing_extended << std::endl;
    }
    left_wing.set_value(left_wing_extended);
    right_wing.set_value(right_wing_extended);
}

bool front_left_wing_extended = false;
bool front_right_wing_extended = false;
bool front_both_wings_extended = false;
void extend_front_wings() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        front_both_wings_extended = !front_both_wings_extended;
        front_left_wing_extended = front_both_wings_extended;
        front_right_wing_extended = front_both_wings_extended;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        front_left_wing_extended = !front_left_wing_extended;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        front_right_wing_extended = !front_right_wing_extended;
        std::cout << right_wing_extended << std::endl;
    }
    left_front_wing.set_value(front_left_wing_extended);
    right_front_wing.set_value(front_right_wing_extended);
}

void extend_left_wing() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        left_wing_extended = !left_wing_extended;
    }
    left_wing.set_value(left_wing_extended);
}

void extend_right_wing() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
       right_wing_extended = !right_wing_extended;
    }
    right_wing.set_value(right_wing_extended);
}

bool blocker_extended = false;
void extend_blocker() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {  blocker_extended = !blocker_extended; }
    blocker.set_value(blocker_extended);
}

bool cata_toggled = false;
bool arm_down = false;
bool down_flywheel_enabled = true;
bool arm_down_on_single_button = true;
int counter_cata = 0;

void raw_cata(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        cata_toggled = !cata_toggled;
    }
    if (cata_toggled == true) { // we want to move the arm down
         flywheel_arm.move_voltage(-12000); 
         if (cata_sensor.get_value() == 1) {
            flywheel_arm.move_voltage(0);  // once arm is down, stop motor
            arm_down = false;
            down_flywheel_enabled = true;
            cata_toggled = false;
         }
    }
    else if (cata_toggled == false && arm_down == false) {
        flywheel_arm.move_voltage(0); 
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) { // move arm up
        arm_down = true;
        flywheel_arm.move_voltage(-12000); 
    }
    else if (arm_down) {
        counter_cata++;
        if (counter_cata > 100) {
            flywheel_arm.move_voltage(0); 
            arm_down = false;
            down_flywheel_enabled = false;
            counter_cata = 0;
        }
    }
}



bool climb_extended = false;
void extend_climber() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {  climb_extended = !climb_extended; }
    climber.set_value(climb_extended);
}

bool primary_climb_extended = false;
void extend_primary_climber() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {  primary_climb_extended = !primary_climb_extended; }
    primary_climber.set_value(primary_climb_extended);
}

double tbh = 0.0;
double tbhInitial = 0.0;
double tbhGain = 0.0001; 

double downSyndromeSpeed = 300;
double upSyndromeSpeed = 600;

bool flywheel_enabled = false;
void controlFlywheel(int targetVelocity) {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        flywheel_enabled = !flywheel_enabled;
    }
    if (down_flywheel_enabled && flywheel_enabled) { // down
        int currentVelocity = cata_motor.get_actual_velocity();
        int error = downSyndromeSpeed - currentVelocity;

        if ((error > 0 && tbh < 0) || (error < 0 && tbh > 0)) {
            tbh = tbhInitial;
        } else {
            tbh *= (1.0 - tbhGain);
        }

        int motorSpeed = downSyndromeSpeed + (int)tbh;
        cata_motor.move_velocity(motorSpeed);
    }
    else if (down_flywheel_enabled == false && flywheel_enabled) {
        int currentVelocity = cata_motor.get_actual_velocity();
        int error = -upSyndromeSpeed - currentVelocity;

        if ((error > 0 && tbh < 0) || (error < 0 && tbh > 0)) {
            tbh = tbhInitial;
        } else {
            tbh *= (1.0 - tbhGain);
        }

        int motorSpeed = upSyndromeSpeed + (int)tbh;
        cata_motor.move_velocity(-motorSpeed);
    }
    else {
        cata_motor.move_velocity(0);
    }
}

void stop_cata_with_sensor() {
    while (true) {
        if (distance_sensor.get() < 50) {
            flywheel_arm.move_voltage(0);
            break;
        }
        else {
            flywheel_arm.move_voltage(-12000); 
        }
    }
}

bool front_wings_extended = true;
void extend_front_wings_manually() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        front_wings_extended = !front_wings_extended;
    }
    if (front_wings_extended) {
        front_wings.set_value(true);
    } else {
        front_wings.set_value(false);
    }
}

bool odom_piston_extended = true;
void extend_odom_piston() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        odom_piston_extended = !odom_piston_extended;
    }
    odom_piston.set_value(odom_piston_extended);
}

void init_sequence() {
    intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void auton_sequence() {
    intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    cata_motor.move_velocity(-600);
    cata_motor.move_velocity(0);
    stop_cata_with_sensor();
}

void realCataControl() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){ cata_motor.move_voltage(12000); }
    else{ intake_motor.move_voltage(0); }
}

