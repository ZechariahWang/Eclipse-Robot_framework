/**
 * @file Algorithms.cpp
 * @author Zechariah Wang
 * @brief Movement Algorithms involving global cartesian coordinates. Includes MTP, MTRP, Boomerang, and TTP/STP
 * @version 0.1
 * @date 2023-07-04
 * 
 */

#include "main.h"
#include "vector"
#include "array"
#include "iostream"
#include "algorithm"

/**
 * @brief Motion Algorithm Class Helper Functions move to point class material theme ocean
 * 
 */

using namespace Eclipse;

void Eclipse::FeedbackControl::set_constants(const double t_kp, const double r_kp, const double f_tt, const double t){ // Set constants
  mtp.t_kp = t_kp;
  mtp.r_kp = r_kp;
  mtp.target_final_tol = f_tt;
  mtp.target_tol = t;
}

void Eclipse::FeedbackControl::reset_mtp_constants(){ // Reset values
  mtp.distance = 0;
  mtp.alpha = 0;
  mtp.t_error = 0;
  mtp.beta = 0;
  mtp.iterator = 0;
}

Eclipse::FeedbackControl::FeedbackControl(){
  mtp.t_local_kp = 13;
  mtp.t_local_derivative          = 0;
  mtp.t_local_integral            = 0;
  mtp.t_local_tolerance           = 3;
  mtp.t_local_error               = 0;
  mtp.t_local_previouserror       = 0;
  mtp.t_local_multiplier          = 3000;
  mtp.t_local_averageposition     = 0;
  mtp.t_local_averageHeading      = 0;
  mtp.t_local_FailSafeCounter     = 0;
  mtp.t_local_threshholdcounter   = 0;
}

/**
 * @brief MTP Algorithm. Move to a desired coordinate position while facing a desired angle. Can only be used with mecanum drives
 * 
 * @param targetX the target x coordinate
 * @param targetY the target y coordinate
 * @param targetTheta the target angle in degrees
 * @param translationSpeed max movement speed
 * @param rotationSpeed max rotation speed
 */ 

double local_rotation_pid(double t_theta){
  utility::fullreset(0, false);
  mtp.t_local_error = 0;
  mtp.t_local_previouserror = 0;
  mtp.t_local_integral = 0;
  mtp.t_local_derivative = 0;
  mtp.t_local_FailSafeCounter = 0;
  mtp.t_local_averageHeading = imu_sensor.get_rotation(); 
  mtp.t_local_error = t_theta - mtp.t_local_averageHeading; 
  mtp.t_local_integral += mtp.t_local_error; 
  if (mtp.t_local_error == 0 || mtp.t_local_error > t_theta) { mtp.t_local_integral = 0; }
  mtp.t_local_derivative = mtp.t_local_error - mtp.t_local_previouserror; 
  mtp.t_local_previouserror = mtp.t_local_error;
  double voltage = (mtp.t_local_error * mtp.t_kp * 0.01) * 94; 
  if(fabs(mtp.t_local_error) < mtp.t_local_tolerance){ mtp.t_local_threshholdcounter++; }
  else{ mtp.t_local_threshholdcounter = 0; }
  if (fabs(mtp.t_local_error - mtp.t_local_previouserror) < 0.3) { mtp.t_local_FailSafeCounter++; }
  else { mtp.t_local_FailSafeCounter = 0; }
  return voltage;
}

void Eclipse::FeedbackControl::simultaneous_mov_executor(double targetX,
                                                    double targetY,
                                                    double targetTheta,
                                                    double translationSpeed,
                                                    double rotationSpeed){

  double previousDriveError = 0; double previousTurnError = 0;
  while (true){
    double theta = current_robot_heading() * M_PI / 180;
    double driveError = sqrt(pow(targetX - global_robot_x, 2) + pow(targetY - global_robot_y, 2));
    double positionHypo = sqrt(pow(global_robot_x, 2) + pow(global_robot_y, 2));
    double driveOutput = (driveError * 8) + ((driveError - previousDriveError) * 1.3);

    double turnError = (-theta - targetTheta);
    turnError = atan2f(sinf(turnError), cosf(turnError));
    double turnOutput = local_rotation_pid(targetTheta);

    double angleDesired = atan2f(targetX - global_robot_x, targetY - global_robot_y);
    double angleDrive = (angleDesired - theta);
    angleDrive = atan2f(sinf(angleDrive), cosf(angleDrive));

    double velDrive = driveOutput * cos(angleDrive); 
    double velStrafe = driveOutput * sin(angleDrive);

    double speedFL; double speedBL;
    double speedFR; double speedBR;

    if(fabs(driveError) < 5 && fabs(turnError) < 3){ utility::stop(); break; }
    else{
      speedFL = velDrive + velStrafe + turnOutput; speedBL = velDrive - velStrafe + turnOutput;
      speedFR = velDrive - velStrafe - turnOutput; speedBR = velDrive + velStrafe - turnOutput;
    }

    dt_front_left.move_velocity((speedFL)); dt_rear_left.move_velocity((speedBL));
    dt_front_right.move_velocity((speedFR)); dt_rear_right.move_velocity((speedBR));

    previousTurnError = turnError; 
    previousDriveError = driveError;
    pros::delay(10);
  }
}

/**
 * @brief move to a specific position at a specific angle
 * 
 * @param targetX the target x coordinate
 * @param targetY the target y coordinate
 * @param targetHeading the target angle in degrees
 * @param radius the radius of the arc
 */

void Eclipse::FeedbackControl::move_to_reference_pose(const double targetX,
                                                 const double targetY,
                                                 const double targetHeading,
                                                 const double radius){
  mtp.reset_mtp_constants();
  while (true){
    double abstargetAngle = atan2f(targetX - global_robot_x, targetY - global_robot_y) * 180 / M_PI;
    if (abstargetAngle < 0){ abstargetAngle += 360; }

    mtp.distance = sqrt(pow(targetX - global_robot_x, 2) + pow(targetY - global_robot_y, 2));
    mtp.alpha = utility::getAngleError(abstargetAngle, targetHeading, false);
    mtp.t_error = utility::getAngleError(abstargetAngle, current_robot_heading(), false);
    mtp.beta = atan(radius / mtp.distance) * 180 / M_PI;

    if (alpha < 0){ beta = -beta;}
    if (fabs(alpha) < fabs(beta)){ mtp.r_error = mtp.t_error + alpha; }
    else{ mtp.r_error = mtp.t_error + beta; }

    if (mtp.r_error > 180 || mtp.r_error < -180){ mtp.r_error = mtp.r_error - (utility::sgn(mtp.r_error) * 360); }

    double linearVel = mtp.t_kp * mtp.distance;
    double turnVel = mtp.r_kp * mtp.r_error;
    double closetoTarget = false;

    if (mtp.distance < mtp.target_tol){ closetoTarget = true;}
    if (closetoTarget){
      linearVel = mtp.t_kp * mtp.distance * utility::sgn(cos(mtp.r_error * M_PI / 180));
      mtp.r_error = utility::getAngleError(targetHeading, current_robot_heading(), false);
      turnVel = mtp.r_kp * atan(tan(mtp.r_error * M_PI / 180)) * 180 / M_PI;
    }

    int16_t left_volage = linearVel + turnVel;
    int16_t right_voltage = linearVel - turnVel;
    int16_t linError_f = sqrt(pow(targetX - global_robot_x, 2) + pow(targetY - global_robot_y, 2));

    utility::leftvoltagereq(left_volage * (12000.0) / 127);
    utility::rightvoltagereq(right_voltage * (12000.0 / 127));

    if (fabs(sqrt(pow(targetX - global_robot_x, 2) + pow(targetY - global_robot_y, 2))) < mtp.target_final_tol)
    { 
      utility::stop();
      break;
    }
    else {mtp.iterator = 0;}
    if (mtp.iterator > 10) {
      utility::stop();
      break;
    }
    pros::delay(10);
  }
}

/**
 * @brief Turn to a specific coordinate position
 * 
 * @param targetX the target x coordinate
 * @param targetY the target y coordinate
 */

void Eclipse::FeedbackControl::TurnToPoint(const int targetX, const int targetY){
  double counter = 0;
  while (true){
    double angular_error = utility::get_angular_error(targetX, targetY);
    double angular_speed = angular_error * 5;

    utility::leftvoltagereq(angular_speed * (12000.0 / 127) * 5);
    utility::rightvoltagereq(-angular_speed * (12000.0 / 127) * 5);
    std::cout << "in ttp loop" << std::endl;

    if (fabs(angular_error) < 3){
      counter++;
    }
    if (counter >= 10){
      utility::stop();
      counter = 0;
      return;
    }
    pros::delay(10);
  }
}

/**
 * @brief Mimic the movement of the standard MTP algorithm. Used for Pure Pursuit
 * 
 * @param target_x the target x coordinate
 * @param target_y the target y coordinate
 * @param max_linear_speed max speed robot can move while doing lateral movements
 * @param max_rotation_speed max speed robot can move while doing rotational movements
 * @param kp_linear linear proportional value
 * @param kp_angular angular proportional value
 */

void mimic_move_to_point(double target_x,
                   double target_y,
                   double max_linear_speed,
                   double max_rotation_speed, 
                   double kp_linear, 
                   double kp_angular){

    double angular_error = utility::get_angular_error(target_x, target_y);
    double linear_error = utility::get_distance_error(target_x, target_y);

    double linear_speed = linear_error * kp_linear;
    double angular_speed = angular_error * kp_angular;

    if (linear_speed > max_linear_speed) {
      linear_speed = max_linear_speed;
    }
    if (linear_speed < -max_linear_speed){
      linear_speed = -max_linear_speed;
    }
    if (angular_speed > max_rotation_speed){
      angular_speed = max_rotation_speed;
    }
    if (angular_speed < -max_rotation_speed){
      angular_speed = -max_rotation_speed;
    }

    if (fabs(linear_error) < 7){
      utility::stop();
      return;
    }

    utility::leftvoltagereq((linear_speed + angular_speed) * (12000.0 / 127));
    utility::rightvoltagereq((linear_speed - angular_speed) * (12000.0 / 127));
}

/**
 * @brief Move to desired gloal robot position
 * 
 * @param target_x the target x coordinate
 * @param target_y the target y coordinate
 * @param max_linear_speed max speed robot can move while doing lateral movements
 * @param max_rotation_speed max speed robot can move while doing rotational movements
 * @param kp_linear linear proportional value
 * @param kp_angular angular proportional value
 */

// TODO work on reverse movement
int ct = 0;
void Eclipse::FeedbackControl::move_to_point(
                   double target_x,
                   double target_y,
                   double max_linear_speed,
                   double max_rotation_speed, 
                   double kp_linear, 
                   double kp_angular,
                   bool backwards){

  while (true){

    odom.update_odom();

    double angular_error = utility::getAngleError(target_x, target_y, false);
    double linear_error = utility::getDistanceError(target_x, target_y);

    if (backwards == true){
      angular_error = utility::getAngleError(target_x, target_y, true);
      linear_error = -linear_error;
    }
    double linear_speed = linear_error * kp_linear;
    double angular_speed = angular_error * kp_angular;

    if (ct < 20){
      linear_speed = 0;
    }

    std::cout << "angular error: " << angular_error << std::endl;

    utility::engage_left_motors((linear_speed - angular_speed) * (12000.0 / 127));
    utility::engage_right_motors((linear_speed + angular_speed) * (12000.0 / 127));
    ct++;
  
    if (fabs(linear_error) < 3){
      utility::motor_deactivation();
      break;
    }


    pros::delay(10);
  }
}

void boomerang(double target_x, double target_y, double target_theta, double max_linear_speed, double max_rotation_speed, double d_lead, double kp_linear, double kp_angular, bool reverse) {
    bool settling = false;
    double settling_error = 7.5;
    double minError = 5;
    double minRotationError = 0.052;
    double prev_lin_error = 100;
    double prev_lin_speed;
    if (reverse){ target_theta += 180; }
    while (true) {
      odom.update_odom();
      bool noPose = (target_theta > 360);
      double carrotPoint_x = 0;
      double carrotPoint_y = 0;

      if (fabs(utility::getDistanceError(target_x, target_y)) < settling_error && settling == false){
        std::cout << "i am settling" << std::endl;
        settling = true;
        max_linear_speed = fmax(fabs(prev_lin_speed), 30);
      }

      if (noPose == false){
        double h = utility::getDistanceError(target_x, target_y);
        double at = target_theta * M_PI / 180.0;
        carrotPoint_x = target_x - h * cos(at) * d_lead;
        carrotPoint_y = target_y - h * sin(at) * d_lead;
      } else {
        carrotPoint_x = target_x;
        carrotPoint_y = target_y;
      }

      if (settling){
        carrotPoint_x = target_x;
        carrotPoint_y = target_y;
      }

      // get current error
      double lin_error = utility::getDistanceError(target_x, target_y);
      double ang_error = utility::getAngleError(carrotPoint_x, carrotPoint_y, false);
      double global_ang_error = utility::getAngleError(target_x, target_y, false);

      if (settling){
        ang_error = utility::getAngleError(target_x, target_y, false);
      }

      if (reverse == true){
        lin_error = -lin_error;
        ang_error = utility::getAngleError(carrotPoint_x, carrotPoint_y, true);
      }

      // calculate linear speed
      double lin_speed;
      lin_speed = lin_error * kp_linear;
      double ang_speed = ang_error * kp_angular;

      double over_turn = fabs(lin_speed) + fabs(ang_speed) - max_linear_speed;
      if (over_turn > 0){
        if (lin_speed > 0) {
          lin_speed -= over_turn;
        }
        else {
          lin_speed += over_turn;
        }
      }

      // cap linear speed
      if ((lin_speed * (12000.0 / 127)) > max_linear_speed * (12000.0 / 127)) {
        lin_speed = max_linear_speed;
      }
      else if (lin_speed * (12000.0 / 127) < -max_linear_speed * 12000.0 / 127){
        lin_speed = -max_linear_speed;
      }
      if ((ang_speed * (12000.0 / 127)) > max_rotation_speed * (12000.0 / 127)) {
        ang_speed = max_rotation_speed;
      }
      else if (ang_speed * (12000.0 / 127) < -max_rotation_speed * 12000.0 / 127){
        ang_speed = -max_rotation_speed;
      }

      // add speeds together zech has autism
      double left_speed = lin_speed - ang_speed;
      double right_speed = lin_speed + ang_speed;

      prev_lin_error = lin_error;
      prev_lin_speed = lin_speed;

      utility::engage_left_motors(left_speed * (12000.0 / 127));
      utility::engage_right_motors(right_speed * (12000.0 / 127));

      if (fabs(lin_error) < minError){
        utility::motor_deactivation();
        break;
      }

      pros::delay(10);
    }
}

std::vector<std::pair<double, double>> test_path = {
  {0.0, 0.0},
  {5, 10},
  {2, 10},
  {3, 5},
  {10, 8},
  {3, 6},
  {2, 12},
  {5, 19},
};

double kp_linear = 3;
double kp_angular = 1.5;

void eclipse_trajectory_algorithm(std::vector<std::pair<double, double>> path, double max_linear_speed, double max_rotation_speed){
  for (size_t i = 0; i < path.size(); i++) {

    const auto& point = path[i];
    std::cout << "x: " << point.first << ", y: " << point.second << std::endl;

    if (i == path.size() - 1){
      mtp.move_to_point(point.first, point.second, max_linear_speed, max_rotation_speed, kp_linear, kp_angular, false);
      return;
    }

    while (true){
      double angular_error = utility::get_angular_error(point.first, point.second);
      double linear_error = utility::get_distance_error(point.first, point.second);

      double linear_speed = 5;
      double angular_speed = angular_error * 5;

      if (linear_speed > max_linear_speed) {
        linear_speed = max_linear_speed;
      } if (linear_speed < -max_linear_speed){
        linear_speed = -max_linear_speed;
      } if (angular_speed > max_rotation_speed){
        angular_speed = max_rotation_speed;
      } if (angular_speed < -max_rotation_speed){
        angular_speed = -max_rotation_speed;
      }

      utility::leftvoltagereq((linear_speed + angular_speed) * (12000.0 / 127));
      utility::rightvoltagereq((linear_speed - angular_speed) * (12000.0 / 127));

      data_displayer.output_game_data(); // Display robot stats and info
      data_displayer.display_data();
      data_displayer.output_misc_data();

      if (fabs(linear_error) < 5){
        break;
      }

      pros::delay(10);
    }
    std::cout << "point reached" << std::endl;
  }
}

/**
 * @brief Move the chassis towards the target pose
 *
 * Uses the boomerang controller
 *
 * @param x x location
 * @param y y location
 * @param theta theta (in degrees). Target angle
 * @param timeout longest time the robot can spend moving
 * @param async whether the function should be run asynchronously. false by default
 * @param forwards whether the robot should move forwards or backwards. true for forwards (default), false for
 * backwards
 * @param lead the lead parameter. Determines how curved the robot will move. 0.6 by default (0 < lead < 1)
 * @param chasePower higher values make the robot move faster but causes more overshoot on turns. 0 makes it
 * default to global value
 * @param maxSpeed the maximum speed the robot can move at. 127 at default
 * @param log whether the chassis should log the turnTo function. false by default
 */

void boomerang_in_radians(float x, float y, float theta, bool forwards, float lead, float maxSpeed) {
    double target_x = x;
    double target_y = y;
    double target_theta = M_PI_2 - math.deg_to_rad(theta);

    double last_x = global_robot_x;
    double last_y = global_robot_y;
    double last_theta = heading;
    double prevLinearPower = 0;
    int distTravelled = 0;
    bool close = false; 

    if (!forwards) target_theta = fmod(target_theta + M_PI, 2 * M_PI); // backwards movement

    while (true) {
      // update_odom_new();
        // get current pose
        double current_x = global_robot_x;
        double current_y = global_robot_y;
        double current_theta = heading;
        double h = std::sqrt(pow(target_x - global_robot_x, 2) + pow(target_y - global_robot_y, 2));
        if (!forwards) current_theta += M_PI;
        current_theta = M_PI_2 - current_theta; // convert to standard form

        // update completion vars
        distTravelled += std::sqrt(pow(last_x - global_robot_x, 2) + pow(last_y - global_robot_y, 2));
        last_x = global_robot_x;
        last_y = global_robot_y;
        last_theta = current_theta;

        // check if the robot is close enough to the target to start settling
        if (std::sqrt(pow(target_x - global_robot_x, 2) + pow(target_y - global_robot_y, 2) < 7.5 && close == false)) {
            close = true;
            maxSpeed = fmax(fabs(prevLinearPower), 30);
        }

        double carrot_x = target_x - h * std::cos(math.deg_to_rad(target_theta)) * lead;
        double carrot_y = target_y - h * std::sin(math.deg_to_rad(target_theta)) * lead;
        if (close){
          carrot_x = x;
          carrot_y = y;
        }

        // calculate error
        float angularError = utility::get_angular_error(carrot_x, carrot_y);
        float linearError = utility::get_distance_error(carrot_x, carrot_y) * cos(math.deg_to_rad(angularError)); // linear error
        if (close) angularError = utility::get_min_angle_error(target_theta, current_robot_heading(), false); // settling behavior
        if (!forwards) linearError = -linearError;

        // get PID outputs
        float angularPower = math.rad_to_deg(angularError) * 5;
        float linearPower = linearError * 5;


        // prioritize turning over moving
        float overturn = fabs(angularPower) + fabs(linearPower) - maxSpeed;
        if (overturn > 0) linearPower -= linearPower > 0 ? overturn : -overturn;
        prevLinearPower = linearPower;

        // calculate motor powers
        float leftPower = linearPower + angularPower;
        float rightPower = linearPower - angularPower;

        // move the motors
        utility::leftvoltagereq(leftPower);
        utility::rightvoltagereq(rightPower);

        if (fabs(h) < 3){
          utility::leftvoltagereq(0);
          utility::rightvoltagereq(0);
          return;
        }

        pros::delay(10); // delay to save resources
    }
    distTravelled = -1;
}

void new_boomerang(float x, float y, float theta, float lead) {
    double target_theta = 90 - theta;
    bool close = false; // used for settling
    double distTravelled = 0;
    while (true) {
        // get current pose
        // update_odom_new();
        double temp_odom_heading = heading;
        temp_odom_heading = 90.0 - (temp_odom_heading * 180 / M_PI); // convert to standard form in degrees

        // check if the robot is close enough to the target to start settling
        double h = std::sqrt(pow(x - global_robot_x, 2) + pow(y - global_robot_y, 2));
        if (h < 7.5) close = true;

        // calculate the POOP point

        double carrot_x = x - h * std::cos(math.deg_to_rad(theta)) * lead;
        double carrot_y = y - h * std::sin(math.deg_to_rad(theta)) * lead;
        if (close){
          carrot_x = x; // settling behavior
          carrot_y = y;
        }

        mimic_move_to_point(carrot_x, carrot_y, 900, 900, 3, 1.5);

        if (fabs(h) < 5){
          utility::stop();
          return;
        }
    }
  }






