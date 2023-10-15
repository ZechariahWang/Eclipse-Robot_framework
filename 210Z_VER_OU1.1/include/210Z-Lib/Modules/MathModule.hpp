#include "iostream"

namespace Eclipse{
    class Math{
        public:
            int32_t rad_to_deg(const double angle);
            int32_t deg_to_rad(const double angle);
            double signum(const double num);
            double get_linear_distance_error(std::pair<double, double> start_point, std::pair<double, double> end_point);
            double clamp_num(const double num, const double min, const double max);
            double get_ticks_per_inch(const double wheel_diameter, const double cartridge, const double ratio);
            int32_t find_min_error_wrapped(int16_t target_angle, int16_t current_angle);
            double get_abs_target_angle(std::pair<double, double> start_point, std::pair<double, double> end_point);
            double get_min_angle_error(float angle1, float angle2, bool radians);
            std::pair<double, double> quadratic_formula(double a, double b, double c);

    };
}
