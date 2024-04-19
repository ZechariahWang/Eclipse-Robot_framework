/**
 * @file Odometry.cpp
 * @author Zechariah Wang
 * @brief Odometry logic for global position tracking within robot using cartesian coordinates
 * @version 0.1
 * @date 2023-07-4
 * 
 */

#include "main.h"
#include "vector"
#include "variant"
#include "array"

double    global_robot_x; // global X
double    global_robot_y; // global Y
double    globalTheta;    // global theta

using namespace Eclipse;

Position odom_pos;

/**
 * @brief The current theta of the robot wrapped to 360 degrees
 * @return angle wrapped to 360 degrees from raw IMU sensor data
 */

int current_robot_heading() {
  globalTheta = fmod(imu_sensor.get_rotation(), 360);
  while (globalTheta < 0) {
    globalTheta += 360;
  }
  while (globalTheta > 360) {
    globalTheta -= 360;
  }
  return globalTheta; 
}

/**
 * @brief Set the value of drivetrain specifics
 * 
 */

void Eclipse::Odometry::set_horizontal_tracker_specs(double diameter, double offset){
  odom.global_horizontal_diameter = diameter;
  odom.global_horizontal_offset = offset;
}
void Eclipse::Odometry::set_vertical_tracker_specs(double diameter, double offset){
  odom.global_vertical_diameter = diameter;
  odom.global_vertical_offset = offset;
}

double Eclipse::Odometry::get_horizontal_offset(){ return odom.global_horizontal_offset; }
double Eclipse::Odometry::get_vertical_offset() { return odom.global_vertical_offset; }

int auxValue(){ return horizontal_rotation_sensor.get_position() * 3 / 500; }
int forwardVal(){ return -vertical_auxiliary_sensor.get_value(); }

double Eclipse::Odometry::get_distance_travelled(bool vertical, bool horizontal){
  if (vertical == true && horizontal == false){ // Is a vertical sensor aka an encoder wheel
    return (double(-vertical_auxiliary_sensor.get_value()) * odom.global_vertical_diameter * M_PI / 360);
  }
  else if (vertical == false && horizontal == true){ // Is a horizontal sensor aka a rotation sensor
    return (double(-horizontal_rotation_sensor.get_position()) * odom.global_horizontal_diameter * M_PI / 36000);
  }
  else{ // how is the wheel both an encoder and rotation sensor bruh
    std::cout << "ERROR: Wheel type cannot be both an encoder and and rotation sensor " << std::endl;
    return 0;
  }
}

float getEncoderDistanceTraveled() { return (float(vertical_auxiliary_sensor.get_value()) * 2.75 * M_PI / 360); }
float getRotationDistanceTraveled() { return (float(horizontal_rotation_sensor.get_position()) * 2.75 * M_PI / 36000); }
float getMotorDistanceTraveled() { return (float(chassis_right_motors.at(0).get_position()) * 2.75 * M_PI / 360); }

// odom position values
double heading;
 
// previous values
double prev_left_pos = 0;
double prev_right_pos = 0;
double prev_middle_pos = 0;
double prev_heading = 0;

double tpi = 1.14;
double middle_tpi = 1;

// 5.24, 6.3 LEFT
 
void Eclipse::Odometry::update_odom() {
  double right_pos = getMotorDistanceTraveled();
  double middle_pos = getRotationDistanceTraveled();
  // double middle_pos =  0;

  double delta_right = (right_pos - prev_right_pos) / tpi;
  double delta_middle = (middle_pos - prev_middle_pos) / middle_tpi;
  
  double delta_angle;
  heading = -imu_sensor.get_rotation() * M_PI / 180.0;
  delta_angle = heading - prev_heading;
  
  prev_right_pos = right_pos;
  prev_middle_pos = middle_pos;
  prev_heading = heading;
  
  double local_x;
  double local_y;
  
  if (delta_angle) {
    double i = sin(delta_angle / 2.0) * 2.0;
    local_x = (delta_right / delta_angle + (-2.5)) * i; // left to right distance
    local_y = (delta_middle / delta_angle + (6.32)) * i; // middle distance
    // local_x = (delta_right / delta_angle + (-9.54)) * i; // left to right distance
    // local_y = (delta_middle / delta_angle + (0.44)) * i; // middle distance
  } else {
    local_x = delta_right;
    local_y = delta_middle;
  }
  
  double p = heading - delta_angle / 2.0; 
  odom_pos.x += cos(p) * local_x - sin(p) * local_y;
  odom_pos.y += cos(p) * local_y + sin(p) * local_x;

  utility::set_x(odom_pos.x);
  utility::set_y(odom_pos.y);

  data_displayer.output_sensor_data(); 
  data_displayer.display_data();
  data_displayer.output_misc_data();

}

void Eclipse::Odometry::init_odom(){
  pros::Task* tracking_task = nullptr;
  if (tracking_task == nullptr) {
    tracking_task = new pros::Task {[=] {
      while (true) {
        update_odom();
        pros::delay(10);
      }
    }};
  }
}






