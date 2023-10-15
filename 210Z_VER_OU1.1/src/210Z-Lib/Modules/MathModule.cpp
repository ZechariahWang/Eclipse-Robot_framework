/**
 * @file Eclipse::MathModule.cpp
 * @author Zechariah Wang
 * @brief Common Eclipse::Mathematical functions/conversions
 * @version 0.1
 * @date 2023-07-04
 */

#include "main.h"

using namespace Eclipse;

int32_t Eclipse::Math::rad_to_deg(const double angle) { // convert radians to degrees
    return angle * 180 / M_PI;
}

int32_t Eclipse::Math::deg_to_rad(const double angle){ // convert degree values to radians
    return angle * M_PI / 180;
}

double Eclipse::Math::get_linear_distance_error(std::pair<double, double> start_point, std::pair<double, double> end_point){ // simple pythag
    return std::sqrt(pow(end_point.first - start_point.first, 2) + pow(end_point.second - start_point.second, 2));
}

double Eclipse::Math::get_abs_target_angle(std::pair<double, double> start_point, std::pair<double, double> end_point){ // get the absolute target value needed to reach target heading NOT WRAPPED  
    double abs_target_angle = atan2f((end_point.second - start_point.second), (end_point.first - start_point.first)) * 180 / M_PI;
    if (abs_target_angle < 0) { abs_target_angle += 360; }
    return abs_target_angle;
}

double Eclipse::Math::get_min_angle_error(float angle1, float angle2, bool radians){ // find min error needed to reach the target heading when wrapped
    float max = radians ? 2 * M_PI : 360;
    float half = radians ? M_PI : 180;
    angle1 = fmod(angle1, max);
    angle2 = fmod(angle2, max);
    float error = angle1 - angle2;
    if (error > half) error -= max;
    else if (error < -half) error += max;
    return error;
}

double Eclipse::Math::signum(const double num) { // another alternative is the sgn function in utility namesspace
    return (num < 0) ? -1 : ((num > 0) ? 1 : 0);
}

double Eclipse::Math::get_ticks_per_inch(const double wheel_diameter, const double cartridge, const double ratio){ // convert motor values int values to use per inch
  double c = wheel_diameter * M_PI; double tpr = (50.0 * (3600.0 / cartridge) * ratio);
  return (tpr / c);
}

double Eclipse::Math::clamp_num(const double num, const double min, const double max){ // Keep num within the desired min and max val 
    if (num > max){ return max; }
    if (num < min) { return min; }
    else { return num; }
}

int32_t Eclipse::Math::find_min_error_wrapped(int16_t target_angle, int16_t current_angle){ // With robot global heading wrapped, what is the fastest way to reach the target heading?
  double turn_angle = target_angle - current_angle;
  if (turn_angle > 180 || turn_angle < -180) { turn_angle = turn_angle - (utility::sgn(turn_angle) * 360); }
  return turn_angle;
}

std::pair<double, double> Eclipse::Math::quadratic_formula(double a, double b, double c){ // calculate all solutions to quadratic
    double discriminant = b * b - 4 * a * c;

    if(discriminant < 0){
        throw std::runtime_error("QF: NO REAL SOL");
    }
    else if(fabs(discriminant) <= 0.001){
        return {-b / 2 * a, -b / 2 * a};
    }
    else{
        double x1 = (-b + sqrt(discriminant)) / (2 * a);
        double x2 = (-b - sqrt(discriminant)) / (2 * a);

        return {x1, x2};
    }
}
