/**
 * @file CoreAssets.cpp
 * @author Zechariah Wang
 * @brief Cored Op control systems, including dt control and motor type alteration, etc
 * @version 0.1
 * @date 2023-04-07
 */

#include "Config/Globals.hpp"
#include "main.h"

using namespace Eclipse;

const u_int16_t forwardCurve       = 10;
const u_int16_t turnCurve          = 3;
const double euler                 = 2.71828;
static bool toggleRedCurve         = false; // toggle red curve
static bool turningRed             = false; // turning curve
static bool forwardRed             = false; // linear curve

  

/**
 * @brief Set exponential joystick accelerator curve type. Allows for more control of small movements, while maintaining max speed for large movements
 * 
 * @param red Red curve enabled. If not enabled, will use blue graph
 * @param t Joystick damper. Tune value as desired by driver
 * @return the value of the joystick accelerator
 */

int32_t joystick_accelerator(bool red, int8_t input, const double t){
    int16_t value = 0;
    // exponential formula
    if (red) { value = (std::exp(-t / 10) + std::exp((std::abs(input) - 100) / 10) * (1 - std::exp(-t / 10))) * input; } 
    else { value = std::exp(((std::abs(input) - 100) * t) / 1000) * input; }
    return value;
}

/**
 * @brief Raw H-Drive DT Control
 * 
 */

void Eclipse::MatchMovement::dt_Control(){
    int32_t rightXjoystick = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)); // Axis 1
    int32_t rightYjoystick = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)); // Axis 2
    int32_t leftYjoystick  = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)); // Axis 3
    int32_t leftXjoystick  = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)); // Axis 4
    if(abs(leftYjoystick) < 10) leftYjoystick = 0;
    if(abs(rightYjoystick) < 10) rightYjoystick = 0;

    int32_t left = (rightXjoystick + leftYjoystick) * (12000.0 / 127);
    int32_t right = (leftYjoystick - rightXjoystick) * (12000.0 / 127);
    utility::engage_left_motors(left);
    utility::engage_right_motors(right);
}

/**
 * @brief Exponential Joystick curve accelerator
 * 
 */

void Eclipse::MatchMovement::exponential_curve_accelerator(){
    int32_t rightXjoystick = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)); // Axis 1
    int32_t rightYjoystick = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)); // Axis 2
    int32_t leftYjoystick  = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)); // Axis 3
    int32_t leftXjoystick  = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)); // Axis 4
    if(abs(leftYjoystick) < 10) leftYjoystick = 0;
    if(abs(rightYjoystick) < 10) rightYjoystick = 0;

    double turn_val = joystick_accelerator(turningRed, rightXjoystick, turnCurve);
    double forward_val = joystick_accelerator(forwardRed, leftYjoystick, forwardCurve);
    double turnVoltage = turn_val * (12000.0 / 127); 
    double forwardVoltage = forward_val * (12000.0 / 127);
    double left =  forwardVoltage + turnVoltage;
    double right = forwardVoltage - turnVoltage;
    utility::engage_left_motors(left);
    utility::engage_right_motors(right);

}

/**
 * @brief Drivetrain control for holonomic drives
 * 
 */

void Eclipse::MatchMovement::x_drive_dt_Control(){
  double front_left  = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) + (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
  double back_left   = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) - (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
  double front_right = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) - (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
  double back_right  = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) + (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
        
  double max_raw_sum = (double)(abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) + abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)));
  double max_XYstick_value = (double)(std::max(abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)), abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X))));
        
  if (max_raw_sum != 0) {
    front_left  = front_left / max_raw_sum * max_XYstick_value;
    back_left   = back_left / max_raw_sum * max_XYstick_value;
    front_right = front_right / max_raw_sum * max_XYstick_value;
    back_right  = back_right / max_raw_sum * max_XYstick_value;
  }
        
  front_left  = front_left  + controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
  back_left   = back_left   + controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
  front_right = front_right - controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
  back_right  = back_right  - controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
  max_raw_sum = std::max(std::abs(front_left), std::max(std::abs(back_left), std::max(std::abs(front_right), std::max(std::abs(back_right), 100.0))));
        
  front_left  = front_left  / max_raw_sum * 100.0;
  back_left   = back_left   / max_raw_sum * 100.0;
  front_right = front_right / max_raw_sum * 100.0;
  back_right  = back_right  / max_raw_sum * 100.0;

  dt_front_left.move_voltage(front_left * (12000.0 / 127));
  dt_rear_left.move_voltage(back_left * (12000.0 / 127));
  dt_front_right.move_voltage(front_right * (12000.0 / 127));
  dt_rear_right.move_voltage(back_right * (12000.0 / 127));
}

/**
 * @brief Force reset coordinate position of robot
 * 
 */

void ForceReset(){ if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){ utility::set_x(0); utility::set_y(0); } }