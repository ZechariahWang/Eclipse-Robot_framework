/**
 * @file ConstantsTuner.cpp
 * @author Zechariah Wang
 * @brief Path Gen for PP with Bezier Curves
 * @version 0.1
 * @date 2023-03-10
 * 
 */

#include "main.h"
#include "pros/misc.h"

void Eclipse::ConstantsTuner::display_constant_data() {
    std::cout << "######################################################################################" << std::endl;
    std::cout << "KP: " << tuner.localKp << std::endl;
    std::cout << "KI: " << tuner.localKi << std::endl;
    std::cout << "KD: " << tuner.localKd << std::endl;
    std::cout << "Global Increment Amount: " << tuner.global_increment_amount << std::endl;
    std::cout << "current_constant_mode: " << tuner.CURRENT_MODE << " | 0 = KP | 1 = Ki | 2 = Kd |" << std::endl;
    std::cout << "######################################################################################" << std::endl;
    std::cout << " " << std::endl;
    pros::delay(500);
}


void Eclipse::ConstantsTuner::modify_desired_constants() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        tuner.localKp -= tuner.global_increment_amount;
        tuner.display_constant_data();
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
        tuner.localKp += tuner.global_increment_amount;
        tuner.display_constant_data();   
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        tuner.localKi -= tuner.global_increment_amount;
        tuner.display_constant_data();
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        tuner.localKi += tuner.global_increment_amount;
        tuner.display_constant_data();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        tuner.localKd -= tuner.global_increment_amount;
        tuner.display_constant_data();
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        tuner.localKd += tuner.global_increment_amount;
        tuner.display_constant_data();
    }
}

void Eclipse::ConstantsTuner::modify_proportional_derivative_increment_amount(double increment_amount) {
    bool wasPressed = false;
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        tuner.global_increment_amount += increment_amount;
        wasPressed = true;
    } if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        tuner.global_increment_amount -= increment_amount;
        wasPressed = true;
    }

    if (tuner.global_increment_amount < 0) { tuner.global_increment_amount = 0; }
    if (wasPressed) { tuner.display_constant_data(); }
}

void Eclipse::ConstantsTuner::shift_constant() {
    bool wasShifted = false;
    double constant_shifter = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0;

    if (constant_shifter >= 1) { tuner.CURRENT_MODE += 1; wasShifted = true; }
    if (constant_shifter <= -1) { tuner.CURRENT_MODE -= 1; wasShifted = true; }

    if (tuner.CURRENT_MODE < 0) { tuner.CURRENT_MODE = 0; wasShifted = true; }
    if (tuner.CURRENT_MODE > 2) { tuner.CURRENT_MODE = 2; wasShifted = true; }

    if (tuner.CURRENT_MODE == 0) {
        tuner.kp_increment_amount = tuner.global_increment_amount;
    } else if (tuner.CURRENT_MODE == 1) {
        tuner.ki_increment_amount = tuner.global_increment_amount;
    } else if (tuner.CURRENT_MODE == 2) {
        tuner.kd_increment_amount = tuner.global_increment_amount;
    }
    if (wasShifted) { tuner.display_constant_data(); }
}

void Eclipse::ConstantsTuner::control_movement_output() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        rot_r.set_r_constants(tuner.localKp, tuner.localKi, tuner.localKd);
        rot_r.set_rotation_pid(360, 90);
    } 
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        rot_r.set_r_constants(tuner.localKp, tuner.localKi, tuner.localKd);
        rot_r.set_rotation_pid(90, 90);
    }
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        rot_r.set_r_constants(tuner.localKp, tuner.localKi, tuner.localKd);
        rot_r.set_rotation_pid(180, 90);
    }
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        rot_r.set_r_constants(tuner.localKp, tuner.localKi, tuner.localKd);
        rot_r.set_rotation_pid(270, 90);
    } 
}

void Eclipse::ConstantsTuner::driver_tuner() {
    tuner.modify_desired_constants();
    tuner.shift_constant();
    tuner.modify_proportional_derivative_increment_amount(1);
    tuner.control_movement_output();
}

