/**
 * @file Eclipse::MetricsModule.cpp
 * @author Zechariah Wang
 * @brief Display robot stats, allow users to better diagnoze robot issues
 * @version 0.1
 * @date 2023-07-04
 */

#include "main.h"

using namespace Eclipse;

/**
 * @brief display core sensor data
 * 
 */

double get_distance_error_new(double d_target_x, double d_target_y){
  double delta_x = d_target_x - utility::get_x();
  double delta_y = d_target_y - utility::get_y();
  return sqrt(delta_x * delta_x + delta_y * delta_y);
}

void Eclipse::Metrics::display_data(){
	char buffer[300];
	sprintf(buffer, SYMBOL_GPS " X: %.2f Y: %.2f Theta: %f", utility::get_x(), utility::get_y(), heading);
	lv_label_set_text(odom_readings_sensor, buffer);
	sprintf(buffer, SYMBOL_WARNING " FL: %.2f BL: %.2f FR: %.2f BR: %.2f", dt_front_left.get_temperature(), dt_rear_left.get_temperature(), dt_front_right.get_temperature(), dt_rear_right.get_temperature());
	lv_label_set_text(dt_readings_sensor, buffer);
	sprintf(buffer, SYMBOL_DRIVE " Sensor 1 Value: NULL ");
	lv_label_set_text(sensor1_readings_sensor, buffer);
	sprintf(buffer, SYMBOL_DRIVE " Sensor 2 Value: NULL");
	lv_label_set_text(sensor2_readings_sensor, buffer);
	sprintf(buffer, SYMBOL_DRIVE " Sensor 3 Value: NULL");
	lv_label_set_text(sensor3_readings_sensor, buffer);
	sprintf(buffer, SYMBOL_DRIVE " Sensor 4 Value: NULL");
	lv_label_set_text(sensor4_readings_sensor, buffer);
}

void Eclipse::Metrics::output_sensor_data(){
    char buffer[300];
	sprintf(buffer, SYMBOL_GPS " X: %.2f Y: %.2f Theta: %f", utility::get_x(), utility::get_y(), heading);
	lv_label_set_text(odom_readings_sensor, buffer);
	sprintf(buffer, SYMBOL_WARNING " FL: %.2f BL: %.2f FR: %.2f BR: %.2f", dt_front_left.get_temperature(), dt_rear_left.get_temperature(), dt_front_right.get_temperature(), dt_rear_right.get_temperature());
	lv_label_set_text(dt_readings_sensor, buffer);
	sprintf(buffer, SYMBOL_DRIVE " Sensor 1 Value: NULL ");
	lv_label_set_text(sensor1_readings_sensor, buffer);
	sprintf(buffer, SYMBOL_DRIVE " Sensor 2 Value: NULL");
	lv_label_set_text(sensor2_readings_sensor, buffer);
	sprintf(buffer, SYMBOL_DRIVE " Sensor 3 Value: NULL");
	lv_label_set_text(sensor3_readings_sensor, buffer);
	sprintf(buffer, SYMBOL_DRIVE " Sensor 4 Value: NULL");
	lv_label_set_text(sensor4_readings_sensor, buffer);
}

/**
 * @brief Display auton selector page
 * 
 */

void Eclipse::Metrics::output_auton_selector(){
    char buffer[300];
	sprintf(buffer, SYMBOL_DRIVE " Selected Path: %d", selected_auton);
	lv_label_set_text(current_auton_display_selector, buffer);
}

/**
 * @brief Display game page
 * 
 */

void Eclipse::Metrics::output_game_data(){
    char buffer[300];
	sprintf(buffer, SYMBOL_GPS " Controller Status %d", controller.is_connected());
	lv_label_set_text(controller_status_game, buffer);
	sprintf(buffer, SYMBOL_WARNING " Battery Capacity: %f", pros::battery::get_capacity());
	lv_label_set_text(battery_percent_game, buffer);
	sprintf(buffer, SYMBOL_DRIVE " Battery Temperature: %d", pros::battery::get_current());
	lv_label_set_text(battery_temp_game, buffer);
	sprintf(buffer, SYMBOL_DRIVE " Time since startup: %d", pros::millis());
	lv_label_set_text(time_since_startup_game, buffer);
	sprintf(buffer, SYMBOL_DRIVE " Competition Status: %d", pros::c::competition_get_status());
	lv_label_set_text(competition_stat_game, buffer);
}

/**
 * @brief Display misc page
 * 
 */

void Eclipse::Metrics::output_misc_data(){
    char buffer[300];
	sprintf(buffer, SYMBOL_DRIVE " lin error: %f", get_distance_error_new(20, 20));
	lv_label_set_text(debug_line1_misc, buffer);
	sprintf(buffer, SYMBOL_DRIVE " carrot_x:  %f", 20 - get_distance_error_new(20, 20) * std::sin(90 * M_PI / 180) * 1);
	lv_label_set_text(debug_line2_misc, buffer);
	sprintf(buffer, SYMBOL_DRIVE " carrot_y %f", 20 - get_distance_error_new(20, 20) * std::cos(90 * M_PI / 180) * 1);
	lv_label_set_text(debug_line3_misc, buffer);
	sprintf(buffer, SYMBOL_DRIVE " gx: ");
	lv_label_set_text(debug_line4_misc, buffer);
	sprintf(buffer, SYMBOL_DRIVE "gy ");
	lv_label_set_text(debug_line5_misc, buffer);
	sprintf(buffer, SYMBOL_DRIVE " Debug Line 6: %f", utility::get_x());
	lv_label_set_text(debug_line6_misc, buffer);
}