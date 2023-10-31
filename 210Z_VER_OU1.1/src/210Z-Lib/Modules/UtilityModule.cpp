/**
 * @file Eclipse::utilityModule.cpp
 * @author Zechariah Wang
 * @brief Helper functions and Eclipse::utility namespace assets
 * @version 0.1
 * @date 2023-07-04
 */

#include "main.h"

/**
 * @brief external Eclipse::utility namespace for common motor voltage requests and distance triangulations 
 * 
 */

int Eclipse::utility::sgn(double num){ return (num < 0) ? -1 : ((num > 0) ? 1 : 0); }
  
double Eclipse::utility::clamp(double num, double min, double max){
  if (num > max){ return max; }
  if (num < min) { return min; }
  else { return num; }
}

void Eclipse::utility::stop(){
  dt_front_left.move_voltage(0); dt_rear_left.move_voltage(0); dt_middle_left.move_voltage(0);
  dt_front_right.move_voltage(0); dt_rear_right.move_voltage(0); dt_middle_right.move_voltage(0);
}

void Eclipse::utility::stop_v(){
  dt_front_left.move_velocity(0); dt_middle_left.move_velocity(0); dt_rear_left.move_velocity(0);
  dt_front_right.move_velocity(0); dt_middle_right.move_velocity(0); dt_rear_right.move_velocity(0); 
}

void Eclipse::utility::leftvelreq(int velocity){ dt_front_left.move_voltage(velocity); dt_middle_left.move_voltage(velocity); dt_rear_left.move_voltage(velocity); }
void Eclipse::utility::rightvelreq(int velocity){ dt_front_right.move_voltage(velocity); dt_middle_right.move_voltage(velocity); dt_rear_right.move_voltage(velocity); }
void Eclipse::utility::leftvoltagereq(double voltage){ dt_front_left.move_voltage(voltage); dt_middle_left.move_voltage(voltage); dt_rear_left.move_voltage(voltage); }
void Eclipse::utility::rightvoltagereq(double voltage){ dt_front_right.move_voltage(voltage); dt_middle_right.move_voltage(voltage); dt_rear_right.move_voltage(voltage); }

void Eclipse::utility::fullreset(double resetval, bool imu){
  dt_front_left.set_zero_position(resetval); dt_middle_left.set_zero_position(resetval); dt_rear_left.set_zero_position(resetval);
  dt_front_right.set_zero_position(resetval); dt_middle_right.set_zero_position(resetval); dt_rear_right.set_zero_position(resetval);
  if (imu == true){ imu_sensor.tare_rotation(); }
}

double Eclipse::utility::get_x(){ return global_robot_x; } 
double Eclipse::utility::get_y(){ return global_robot_y; }

void Eclipse::utility::set_x(double x) { global_robot_x = x; }
void Eclipse::utility::set_y(double y) { global_robot_y = y; }

double Eclipse::utility::get_min_angle_error(float angle1, float angle2, bool radians){
    float max = radians ? 2 * M_PI : 360;
    float half = radians ? M_PI : 180;
    angle1 = fmod(angle1, max);
    angle2 = fmod(angle2, max);
    float error = angle1 - angle2;
    if (error > half) error -= max;
    else if (error < -half) error += max;
    return error;
}

double Eclipse::utility::get_angular_error(double target_x, double target_y){
  double x = target_x - global_robot_x;
  double y = target_y - global_robot_y;
  double delta_theta = atan2(x, y) * 180 / M_PI - current_robot_heading();
  while (fabs(delta_theta) > 180){
    delta_theta -= 360 * delta_theta / fabs(delta_theta);
  }
   return delta_theta;
}

double Eclipse::utility::get_distance_error(double d_target_x, double d_target_y){
  double x = d_target_x;
  double y = d_target_y;
  y -= global_robot_y;
  x -= global_robot_x;
  return sqrt(x * x + y * y);
}

double Eclipse::utility::getAngleError(double target_x, double target_y, bool reverse) {
  double x = target_x;
  double y = target_y;

  x -= global_robot_x;
  y -= global_robot_y;

  double delta_theta = atan2(y, x) - heading;

  // if movement is reversed, calculate delta_theta using a 180 degree rotation
  if (reverse) {
    delta_theta = atan2(-y, -x) - heading;
  }

  while (fabs(delta_theta) > M_PI) {
    delta_theta -= 2 * M_PI * delta_theta / fabs(delta_theta);
  }

  return delta_theta;
}

double Eclipse::utility::getDistanceError(double target_x, double target_y) {
  double x = target_x;
  double y = target_y;

  y -= global_robot_y;
  x -= global_robot_x;
  return sqrt(x * x + y * y);
}

bool Eclipse::utility::is_reversed(int value){
  if (value < 0){ return true; }
  return false;
}

void Eclipse::utility::engage_left_motors(double voltage){
  for (auto i : chassis_left_motors) {
    i.move_voltage(voltage);
  }
}

void Eclipse::utility::engage_right_motors(double voltage){
  for (auto i : chassis_right_motors) {
    i.move_voltage(voltage);
  }
}

void Eclipse::utility::motor_deactivation(){
  for (auto i : chassis_left_motors) {
    i.move_voltage(0);
  }
  for (auto i : chassis_right_motors) {
    i.move_voltage(0);
  }
}

void Eclipse::utility::restart_all_chassis_motors(bool imu_reset){
  for (auto i : chassis_left_motors) {
    i.set_zero_position(0);
  }
  for (auto i : chassis_right_motors) {
    i.set_zero_position(0);
  }
  if (imu_reset){ imu_sensor.tare_rotation(); }
}

double Eclipse::utility::get_encoder_position(){
  double left_values = 0;
  double right_values = 0;
  int left_num = 0; // number of motors
  int right_num = 0; // number of motors

  for (auto i : chassis_left_motors) {
    left_values += i.get_position();
    left_num += 1;
  }
  for (auto i : chassis_right_motors) {
    right_values += i.get_position();
    right_num += 1;
  }

  double avg_left_val = left_values / left_num;
  double avg_right_val = right_values / right_num;

  return (avg_left_val + avg_right_val) / 2;
}




void Eclipse::FeedbackControl::overRideCoordinatePos(double new_gx, double new_gy){ Eclipse::utility::set_x(new_gx); Eclipse::utility::set_y(new_gy); }

