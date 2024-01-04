#include "main.h"
#include <string>


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
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){ intake_motor.move_voltage(12000); }
    else if ((controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))){ intake_motor.move_voltage(-12000); } // intake
    else{ intake_motor.move_voltage(0); }
}

bool left_wing_extended = false;
bool right_wing_extended = false;
bool both_wings_extended = false;
void extend_wings() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        both_wings_extended = !both_wings_extended;
        left_wing_extended = both_wings_extended;
        right_wing_extended = both_wings_extended;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        left_wing_extended = !left_wing_extended;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        right_wing_extended = !right_wing_extended;
        std::cout << right_wing_extended << std::endl;
    }
    left_wing.set_value(left_wing_extended);
    right_wing.set_value(right_wing_extended);
}

void extend_left_wing() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        left_wing_extended = !left_wing_extended;
    }
    left_wing.set_value(left_wing_extended);
}

void extend_right_wing() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
       //  right_wing_extended = !right_wing_extended;
    }
    // right_wing.set_value(right_wing_extended);
}

bool blocker_extended = false;
void extend_blocker() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {  blocker_extended = !blocker_extended; }
    blocker.set_value(blocker_extended);
}

bool cata_toggled = false;
void raw_cata(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) { cata_toggled = !cata_toggled; }
    if (cata_toggled) { cata_motor.move_voltage(-10000); cata_motor_secondary.move_voltage(-10000); }
    else { cata_motor.move_voltage(0); cata_motor_secondary.move_voltage(0); }
}

bool climb_extended = false;
void extend_climber() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {  climb_extended = !climb_extended; }
    climber.set_value(climb_extended);
}

bool odom_piston_extended = false;
void extend_odom_piston(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        odom_piston_extended = !odom_piston_extended;
    }
    odom_piston.set_value(odom_piston_extended);
}


double tbh = 0.0;
double tbhInitial = 0.0;
double tbhGain = 0.001; 

void controlFlywheel(int targetVelocity) {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        int currentVelocity = cata_motor.get_actual_velocity();
        int error = targetVelocity - currentVelocity;

        if ((error > 0 && tbh < 0) || (error < 0 && tbh > 0)) {
            tbh = tbhInitial;
        } else {
            tbh *= (1.0 - tbhGain);
        }

        int motorSpeed = targetVelocity + (int)tbh;
        cata_motor.move_velocity(motorSpeed);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        int currentVelocity = cata_motor.get_actual_velocity();
        int error = -targetVelocity - currentVelocity;

        if ((error > 0 && tbh < 0) || (error < 0 && tbh > 0)) {
            tbh = tbhInitial;
        } else {
            tbh *= (1.0 - tbhGain);
        }

        int motorSpeed = targetVelocity + (int)tbh;
        cata_motor.move_velocity(motorSpeed);
    }
    else {
        cata_motor.move_velocity(0);
    }
}

void stop_cata_with_sensor() {
    while (true) {
        std::cout <<"sensor val: " << distance_sensor.get() << std::endl;
        if (distance_sensor.get() < 30) {
            cata_motor.move_voltage(0);
            cata_motor_secondary.move_voltage(0);
            break;
        }
        else {
            cata_motor.move_voltage(-9000); cata_motor_secondary.move_voltage(-9000); 
        }

        pros::delay(10);

    }
}

