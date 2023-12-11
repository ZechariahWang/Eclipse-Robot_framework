/**
 * @file PID.cpp
 * @author Zechariah Wang
 * @brief Lateral PID logic for translation, angular, curve, and arc movements
 * @version 0.1
 * @date 2023-07-04
 * 
 */

#include "main.h"
#include "vector"
#include "variant"
#include "array"

using namespace Eclipse;

/**
 * @brief Set drivetrain specs
 * 
 * @param n_wheelDiameter the dimater of the drivetrain wheel
 * @param n_gearRatio The gear ratio of the drivetrain
 * @param n_motorCartidge The motor cartridge of the drivetrain
 */

// Set the drivetrain constants (wheel size, motor cartridge, etc)
void Eclipse::TranslationPID::set_dt_constants(const double n_wheelDiameter, const double n_gearRatio, const double n_motorCartridge){
  mov_t.wheelDiameter = n_wheelDiameter;
  mov_t.ratio = n_gearRatio;
  mov_t.cartridge = n_motorCartridge;
}

/**
 * @brief Slew init local data variables
 * 
 */

void Slew::set_slew_min_power(std::vector<double> min_power){
  slew.min_power = min_power;
}

void Slew::set_slew_distance(std::vector<double> distance){
  slew.max_distance = distance;
}

/**
 * @brief Initialize slew data logic
 * 
 * @param slew_enabled Is the slew controller enabled
 * @param max_speec Max speed of slew
 * @param target_pos target end of slew
 * @param current_pos current position of slew
 * @param start init location of slew
 * @param backwards_enabled are we going backwards?
 * @param tpi ticks per inch
 */

void Eclipse::Slew::initialize_slew(bool slew_enabled, const double max_speed, const double target_pos, const double current_pos, const double start, bool backwards_enabled, double tpi){
  slew.enabled = slew_enabled;
  slew.max_speed = max_speed;
  slew.slew_ticks_per_inch = tpi;
  slew.sign = utility::sgn(target_pos - current_pos);
  slew.x_intercept = start + (slew.sign * slew.max_distance[backwards_enabled]) * slew.slew_ticks_per_inch; // gt fix
  slew.y_intercept = max_speed * slew.sign;
  slew.slope = (slew.sign * slew.min_power[backwards_enabled] - slew.y_intercept) / (slew.x_intercept - start);
}

/**
 * @brief PID class constructors
 * 
 */

Eclipse::TranslationPID::TranslationPID(){ // Translation PID Constructor
  mov_t.t_tol = 10;
  mov_t.t_error_thresh = 100;
}

Eclipse::RotationPID::RotationPID(){ // Rotation PID Constructor
  rot_r.r_tol = 10;
  rot_r.r_error_thresh = 3;
}

Eclipse::CurvePID::CurvePID(){ // Curve PID Constructor
  cur_c.c_tol = 20;
  cur_c.c_error_thresh = 3;
}

Eclipse::ArcPID::ArcPID(){ // Arc PID Constructor
  arc_a.a_tol = 20;
  arc_a.a_error_thresh = 10;
}

// Reset core translation variables
void Eclipse::TranslationPID::reset_t_alterables(){
  mov_t.t_derivative = 0;
  mov_t.t_integral = 0;
  mov_t.t_error = 0;
  mov_t.t_prev_error = 0;
  mov_t.t_iterator = 0;
  mov_t.t_failsafe = 0;
}

// Reset core rotation variables
void Eclipse::RotationPID::reset_r_alterables(){
  rot_r.r_derivative = 0;
  rot_r.r_integral = 0;
  rot_r.r_error = 0;
  rot_r.r_prev_error = 0;
  rot_r.r_iterator = 0;
  rot_r.r_failsafe = 0;
}

// Reset core curve variables
void Eclipse::CurvePID::reset_c_alterables(){
  cur_c.c_derivative = 0;
  cur_c.c_integral = 0;
  cur_c.c_error = 0;
  cur_c.c_prev_error = 0;
  cur_c.c_iterator = 0;
  cur_c.c_failsafe = 0;
  cur_c.c_rightTurn = false;
}

// Reset core arc variables
void Eclipse::ArcPID::reset_a_alterables(){
  arc_a.a_derivative = 0;
  arc_a.a_integral = 0;
  arc_a.a_error = 0;
  arc_a.a_prev_error = 0;
  arc_a.a_iterator = 0;
  arc_a.a_failsafe = 0;
  arc_a.a_rightTurn = false;
}

/**
 * @brief PID class constants
 * 
 */

// Set translation PID constants
void Eclipse::TranslationPID::set_t_constants(const double kp, const double ki, const double kd, const double r_kp){
  mov_t.t_kp = kp;
  mov_t.t_ki = ki;
  mov_t.t_kd = kd;
  mov_t.t_h_kp = r_kp;
}

// Set rotation PID constants
void Eclipse::RotationPID::set_r_constants(const double kp, const double ki, const double kd){
  rot_r.r_kp = kp;
  rot_r.r_ki = ki;
  rot_r.r_kd = kd;
}

// Set curve PID constants
void Eclipse::CurvePID::set_c_constants(const double kp, const double ki, const double kd){
  cur_c.c_kp = kp;
  cur_c.c_ki = ki;
  cur_c.c_kd = kd;
}

// Set arc PID constants
void Eclipse::ArcPID::set_a_constants(const double kp, const double ki, const double kd){
  arc_a.a_kp = kp;
  arc_a.a_ki = ki;
  arc_a.a_kd = kd;
}

/**
 * @brief calculate the min angle needed to reach a target theta within 360 degrees
 * 
 * @param targetHeading the target heading of the robot
 * @param currentrobotHeading the current angle held by the robot
 * @return the shortest turn angle needed to reach target theta
 */

double Eclipse::TranslationPID::find_min_angle(int16_t targetHeading, int16_t currentrobotHeading){
  double turnAngle = targetHeading - currentrobotHeading;
  if (turnAngle > 180 || turnAngle < -180) { turnAngle = turnAngle - (utility::sgn(turnAngle) * 360); }
  return turnAngle;
}

/**
 * @brief calculate slew max voltage request
 * 
 * @param current Current position of slew
 * @return Slew speed
 */

double Eclipse::Slew::calculate_slew(const double current){
  if (slew.enabled){
    slew.error = slew.x_intercept - current;
    if (utility::sgn(slew.error) != slew.sign){
      slew.enabled = false;
    }
    else if (utility::sgn(slew.error) == slew.sign){
      return ((slew.slope * slew.error) + slew.y_intercept) * slew.sign;
    }
  }
  return slew.max_speed;
}

/**
 * @brief compute translation PID movement logic
 * 
 * @param current the current position of the robot
 * @param target the desired target of the robot
 * @return PID calculated voltage at that specific position relative to target
 */

double Eclipse::TranslationPID::compute_t(double current, double target){
  mov_t.t_error = target - current;
  mov_t.t_derivative = mov_t.t_error - mov_t.t_prev_error;
  if (mov_t.t_ki != 0){
    mov_t.t_integral += mov_t.t_error;
  }
  if (utility::sgn(mov_t.t_error) !=  utility::sgn(mov_t.t_prev_error)){
    mov_t.t_integral = 0;
  }

  double output = (mov_t.t_kp * mov_t.t_error) + (mov_t.t_integral * mov_t.t_ki) + (mov_t.t_derivative * mov_t.t_kd);
  if (output * (12000.0 / 127) > mov_t.t_maxSpeed * (12000.0 / 127)) output = mov_t.t_maxSpeed;
  if (output * (12000.0 / 127) < -mov_t.t_maxSpeed * (12000.0 / 127)) output = -mov_t.t_maxSpeed;
  mov_t.t_prev_error = mov_t.t_error;
  return output;
}

/**
 * @brief compute rotation PID movement logic
 * 
 * @param current the current raw IMU value of the robot
 * @param target the desired target theta of the robot (IN RAW IMU VALUES)
 * @return PID calculated voltage at that specific angle relative to target angle
 */

double get_min_angle_error_pid(float angle1, float angle2, bool radians){
    float max = radians ? 2 * M_PI : 360;
    float half = radians ? M_PI : 180;
    angle1 = fmod(angle1, max);
    angle2 = fmod(angle2, max);
    float error = angle1 - angle2;
    if (error > half) error -= max;
    else if (error < -half) error += max;
    return error;
}

double Eclipse::RotationPID::compute_r(double current, double target){
  rot_r.r_error = get_min_angle_error_pid(target, current_robot_heading(), false);
  rot_r.r_derivative = rot_r.r_error - rot_r.r_prev_error;
  if (rot_r.r_ki != 0){ rot_r.r_integral += rot_r.r_error; }
  if (rot_r.r_error == 0 || rot_r.r_error > target){ rot_r.r_integral = 0; }

  double output = (rot_r.r_kp * rot_r.r_error) + (rot_r.r_integral * rot_r.r_ki) + (rot_r.r_derivative * rot_r.r_kd);
  if (output * (12000.0 / 127) >= rot_r.r_maxSpeed * (12000.0 / 127)) { output = rot_r.r_maxSpeed; }
  if (output * (12000.0 / 127) <= -rot_r.r_maxSpeed * (12000.0 / 127)) { output = -rot_r.r_maxSpeed; }
  rot_r.r_prev_error = rot_r.r_error;
  return output;
}

/**
 * @brief compute curve PID movement logic
 * 
 * @param current the current raw IMU value of the robot
 * @param target the desired target theta the robot will curve to
 * @return PID calculated voltage at that specific angle relative to target curve
 */

double Eclipse::CurvePID::compute_c(double current, double target){
  cur_c.c_error = get_min_angle_error_pid(target, current_robot_heading(), false);
  cur_c.c_derivative = cur_c.c_error - cur_c.c_prev_error;
  if (cur_c.c_ki != 0){
    cur_c.c_integral += cur_c.c_error;
  }
  if (utility::sgn(cur_c.c_error) !=  utility::sgn(cur_c.c_prev_error)){
    cur_c.c_integral = 0;
  }
  double output = (cur_c.c_kp * cur_c.c_error) + (cur_c.c_integral * cur_c.c_ki) + (cur_c.c_derivative * cur_c.c_kd);

  if (output * (12000.0 / 127) >= cur_c.c_maxSpeed * (12000.0 / 127)) { output = cur_c.c_maxSpeed; }
  if (output * (12000.0 / 127) <= -cur_c.c_maxSpeed * (12000.0 / 127)) { output = -cur_c.c_maxSpeed; }
  cur_c.c_prev_error = cur_c.c_error;
  return output;
}

/**
 * @brief compute arc PID movement logic
 * 
 * @param tx the target global X value calculated from Odometry logic
 * @param ty the target global Y value calculated from Odometry logic
 * @return PID calculated voltage at that specific coordinate vector relative to target vector
 */

double Eclipse::ArcPID::compute_a(double tx, double ty){
  arc_a.a_error = sqrt(pow(tx - utility::get_x(), 2) + pow(ty - utility::get_y(), 2));
  arc_a.a_derivative = arc_a.a_error - arc_a.a_prev_error;
  if (arc_a.a_ki != 0){
    arc_a.a_integral += arc_a.a_error;
  }
  if (utility::sgn(arc_a.a_error) !=  utility::sgn(arc_a.a_prev_error)){
    arc_a.a_integral = 0;
  }
  double output = (arc_a.a_kp * arc_a.a_error) + (arc_a.a_integral * arc_a.a_ki) + (arc_a.a_derivative * arc_a.a_kd);

  if (output * (12000.0 / 127) >= arc_a.a_maxSpeed * (12000.0 / 127)) { output = arc_a.a_maxSpeed; }
  if (output * (12000.0 / 127) <= -arc_a.a_maxSpeed * (12000.0 / 127)) { output = -arc_a.a_maxSpeed; }
  arc_a.a_prev_error = arc_a.a_error;
  return output;
}

/**
 * @brief Driver PID function. Main logic function, combining all translation PID components together
 * 
 * @param target the target translation target position
 * @param maxSpeed the maxspeed the robot may travel at
 */

void Eclipse::TranslationPID::set_translation_pid(double target, 
                                         double maxSpeed, 
                                         bool slew_enabled){

  utility::restart_all_chassis_motors(false); mov_t.reset_t_alterables();
  double TARGET_THETA = current_robot_heading(); double POSITION_TARGET = target; bool is_backwards = false; int8_t cd = 0;
  mov_t.t_maxSpeed = maxSpeed;
  mov_t.circumference = mov_t.wheelDiameter * M_PI;
  mov_t.ticks_per_rev = (50.0 * (3600.0 / mov_t.cartridge) * mov_t.ratio);
  mov_t.ticks_per_inches = (mov_t.ticks_per_rev / mov_t.circumference);
  target *= mov_t.ticks_per_inches;
  double init_left_pos = utility::get_encoder_position();
  kal.update_lateral_components();
  while (true){
    odom.update_odom();
    double avgPos = utility::get_encoder_position();
    kal.lateral_prediction_step();
    double filtered_position = kal.lateral_update_filter_step(avgPos); // filter out robot pos with linear kalman filter
    double avg_voltage_req = mov_t.compute_t(filtered_position, target); // compute proportional integral derivative controller on filtered pos
    double headingAssist = mov_t.find_min_angle(TARGET_THETA, current_robot_heading()) * mov_t.t_h_kp; // apply a heading assist to keep robot moving straight
    // initial timer of 100 ms to keep robot from oscillating
    cd++; if (cd <= 20){ utility::engage_left_motors(0); utility::engage_right_motors(0); continue;}
    if (target < 0) { is_backwards = true; } else { is_backwards = false; }
    double l_output = 0; double r_output = 0;
    if (slew_enabled){
      double c = mov_t.wheelDiameter * M_PI;
      double tpr = (50.0 * (3600.0 / mov_t.cartridge) * mov_t.ratio);
      slew.initialize_slew(true, maxSpeed, target, (utility::get_encoder_position()), init_left_pos, is_backwards, tpr / c);
      double slew_output = slew.calculate_slew(filtered_position);
      double l_output = utility::clamp(avg_voltage_req, -slew_output, slew_output);
      double r_output = utility::clamp(avg_voltage_req, -slew_output, slew_output);
    }
    else{
      l_output = avg_voltage_req;
      r_output = avg_voltage_req;
    }
    utility::engage_left_motors((l_output * (12000.0 / 127)) + headingAssist);
    utility::engage_right_motors((r_output * (12000.0 / 127)) - headingAssist);
    if (fabs(mov_t.t_error) < mov_t.t_error_thresh){ mov_t.t_iterator++; } else { mov_t.t_iterator = 0;}
    if (fabs(mov_t.t_iterator) > mov_t.t_tol){
      utility::motor_deactivation();
      break;
    }
    if (fabs(mov_t.t_error - mov_t.t_prev_error) < 5) mov_t.t_failsafe++;
    if (mov_t.t_failsafe > 3000){
      utility::motor_deactivation();
      break;
    }
    pros::delay(10);
  }
}

/**
 * @brief Driver Rotation PID function. Main logic function, combining all rotation PID components together
 * 
 * @param t_theta the target theta angle
 * @param maxSpeed the maxspeed the robot may make the turn in 
 */

void Eclipse::RotationPID::set_rotation_pid(double t_theta,
                                   double maxSpeed){

  utility::restart_all_chassis_motors(false);
  rot_r.reset_r_alterables();
  rot_r.r_maxSpeed = maxSpeed;
  while (true){
    //odom.update_odom();
    double currentPos = current_robot_heading();
    double vol = rot_r.compute_r(currentPos, t_theta);

    utility::engage_left_motors(vol * (12000.0 / 127));
    utility::engage_right_motors(-vol * (12000.0 / 127));
    if (fabs(rot_r.r_error) < 3) { rot_r.r_iterator++; } else { rot_r.r_iterator = 0;}
    if (fabs(rot_r.r_iterator) >= 10){
      utility::motor_deactivation();
      break;
    }
    if (fabs(rot_r.r_error - rot_r.r_prev_error) < 3) {rot_r.r_failsafe++;}
    if (rot_r.r_failsafe > 3000){
      utility::motor_deactivation();
      break;
    }
    pros::delay(10);
  }
}

/**
 * @brief Driver Curve PID function. Main logic function, combining all curve PID components together
 * 
 * @param t_theta the target theta angle
 * @param maxSpeed the maxspeed the robot may make the turn in 
 * @param curveDamper The amount the swing will be dampered by
 * @param backwards whether or not the robot will make the curve backwards or forwards
 */

void Eclipse::CurvePID::set_curve_pid(double t_theta,
                             double maxSpeed, 
                             double curveDamper,
                             bool backwards){

  utility::restart_all_chassis_motors(false);
  cur_c.reset_c_alterables();
  cur_c.c_maxSpeed = maxSpeed;
  cur_c.c_rightTurn = false;
  while (true){
    double currentPos = imu_sensor.get_rotation();
    double vol = cur_c.compute_c(currentPos, t_theta);

    if (cur_c.c_error > 0){ cur_c.c_rightTurn = true; } else { cur_c.c_rightTurn = false;}
    if (cur_c.c_rightTurn == true && backwards == false){
      utility::engage_left_motors(vol * (12000.0 / 127));
      utility::engage_right_motors(vol * (12000.0 / 127) * curveDamper);
    }
    else if (cur_c.c_rightTurn == false && backwards == false){
      utility::engage_left_motors(fabs(vol) * (12000.0 / 127) * curveDamper);
      utility::engage_right_motors(fabs(vol) * (12000.0 / 127));
    }
    if (cur_c.c_rightTurn == true && backwards == true){
      utility::engage_left_motors(-vol * (12000.0 / 127) * curveDamper);
      utility::engage_right_motors(-vol * (12000.0 / 127));
    }
    else if (cur_c.c_rightTurn == false && backwards == true){
      utility::engage_left_motors(vol * (12000.0 / 127));
      utility::engage_right_motors(vol * (12000.0 / 127) * curveDamper);
    }
    if (fabs(cur_c.c_error) < cur_c.c_error_thresh) { cur_c.c_iterator++; } else { cur_c.c_iterator = 0;}
    if (fabs(cur_c.c_iterator) >= cur_c.c_tol){
      utility::motor_deactivation();
      break;
    }
    if (fabs(cur_c.c_error - cur_c.c_prev_error) < 0.3) {cur_c.c_failsafe++;}
    if (cur_c.c_failsafe > 100000){
      utility::motor_deactivation();
      break;
    }
    pros::delay(10);
  }
}

/**
 * @brief Driver Arc PID function. Main logic function, combining all arc PID components together
 * 
 * @param t_x the target X position coordinate
 * @param t_y the target Y position coordinate
 * @param maxSpeed the max speed the robot may make the turn in 
 * @param arcDamper the amount the arc movement will be dampered by
 */

void Eclipse::ArcPID::set_arc_pid(double t_x, 
                         double t_y, 
                         double maxSpeed, 
                         double arcDamper){
                          
  utility::restart_all_chassis_motors(false);
  arc_a.reset_a_alterables();
  arc_a.a_maxSpeed = maxSpeed;
  arc_a.a_rightTurn = false;
  while (true){
    double currentPos = imu_sensor.get_rotation();
    double vol = arc_a.compute_a(t_x, t_y);

    if (arc_a.a_error >= 0){ arc_a.a_rightTurn = true; } else { arc_a.a_rightTurn = false;}
    if (arc_a.a_rightTurn){
      utility::engage_left_motors(vol * (12000.0 / 127));
      utility::engage_right_motors(vol * (12000.0 / 127) * arcDamper);
    }
    else if (arc_a.a_rightTurn == false){
      utility::engage_left_motors(fabs(vol) * (12000.0 / 127) * arcDamper);
      utility::engage_right_motors(fabs(vol) * (12000.0 / 127));
    }
    if (fabs(arc_a.a_error) < arc_a.a_error_thresh) { arc_a.a_iterator++; } else { arc_a.a_iterator = 0;}
    if (fabs(arc_a.a_iterator) >= arc_a.a_tol){
      utility::motor_deactivation();
      break;
    }
    if (fabs(arc_a.a_error - arc_a.a_prev_error) < 0.3) {arc_a.a_failsafe++;}
    if (arc_a.a_failsafe > 100000){
      utility::motor_deactivation();
      break;
    }
    pros::delay(10);
  }
}



