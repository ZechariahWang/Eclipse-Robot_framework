/**
 * @file InitializeModule.cpp
 * @author Zechariah Wang
 * @brief Auton selector, and pre-match initialization
 * @version 0.1
 * @date 2023-07-04
 */

#include "main.h"
#include "map"
#include "pros/misc.h"
#include "string"

using namespace Eclipse;

uint8_t auton_finalized; // Final auton choice
uint8_t selected_auton; // Auton choice
uint8_t global_auton; // Auton choice

/**
 * @brief get input from auton selector
 * 
 */

void Eclipse::Selector::receive_selector_input(u_int32_t time){
    Metrics data_displayer;
    char buffer2[300];
    int64_t current_time = 0; u_int16_t iterator = 1; 
    while (current_time <= time){
    	data_displayer.display_data();
        if (auton_finalized == 1){
        	sprintf(buffer2, "Chosen Auton: %d", selected_auton);
	        lv_label_set_text(current_auton_display_selector, buffer2);
		    pros::delay(2000);
        	sprintf(buffer2, "Entering game phase...");
	        lv_label_set_text(current_auton_display_selector, buffer2);
            pros::delay(3000);
            break;
        }
        pros::delay(10);
    }
}

/**
 * @brief call selected auton
 * 
 */

void Eclipse::Selector::select_current_auton(){
    int16_t chosenAuton = selected_auton;
    switch (chosenAuton) {
    case 0:  global_auton = 0;  AutonSelectorPrimary(0);       break;
    case 1:  global_auton = 1;  AutonSelectorPrimary(1);       break;
    case 2:  global_auton = 2;  AutonSelectorPrimary(2);       break;
    case 3:  global_auton = 3;  AutonSelectorPrimary(3);       break;
    case 4:  global_auton = 4;  AutonSelectorPrimary(4);       break;
    case 5:  global_auton = 5;  AutonSelectorPrimary(5);       break;
    case 6:  global_auton = 6;  AutonSelectorPrimary(6);       break;
    case 7:  global_auton = 7;  AutonSelectorPrimary(7);       break;
    case 8:  global_auton = 8;  AutonSelectorPrimary(8);       break;
    case 9:  global_auton = 9;  AutonSelectorPrimary(9);       break;
    case 10: global_auton = 10; AutonSelectorPrimary(10);      break;
    default: global_auton = 0;  AutonSelectorPrimary(0);       break;
    }
}

/**
 * @brief reset all vital sensors
 * 
 */

void Eclipse::Selector::reset_all_primary_sensors(){
    imu_sensor.tare_rotation();
    horizontal_rotation_sensor.reset_position();
    dt_front_left.set_zero_position(0);
    dt_front_right.set_zero_position(0);
    dt_rear_left.set_zero_position(0);
    dt_rear_right.set_zero_position(0);
    dt_middle_left.set_zero_position(0);
    dt_middle_right.set_zero_position(0);
    utility::set_x(0); utility::set_y(0);
}
