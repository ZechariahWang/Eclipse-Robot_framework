/**
 * @file SecondaryOpDriver.cpp
 * @author Zechariah Wang
 * @brief Driver functions for op control in main
 * @version 0.1
 * @date 2023-04-05
 */

#include "main.h"

void DRIVER_PHASE(double competition_start_time) {
	op_mov.exponential_curve_accelerator();
	odom.update_odom();

	power_intake();
	extend_wings();
	extend_front_wings();
	extend_odom_piston();
	extend_climber();
	extend_primary_climber(competition_start_time);
	pros::delay(10); // Dont hog CPU ;)
}

void CONSTANT_TUNER_PHASE() { tuner.driver_tuner(); pros::delay(10); }