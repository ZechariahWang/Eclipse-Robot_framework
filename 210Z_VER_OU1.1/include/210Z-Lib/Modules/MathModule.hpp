#include "iostream"

namespace Eclipse{
    class Math{
        public:
            double rad_to_deg(const double angle);
            double deg_to_rad(const double angle);
            double signum(const double num);
            double get_linear_distance_error(double start_x, double start_y, double end_x, double end_y);
            double clamp_num(const double num, const double min, const double max);
            double get_ticks_per_inch(const double wheel_diameter, const double cartridge, const double ratio);
            int32_t find_min_error_wrapped(int16_t target_angle, int16_t current_angle);
            double get_abs_target_angle(double start_x, double start_y, double end_x, double end_y);
            double get_min_angle_error(float angle1, float angle2, bool radians);
            std::pair<double, double> quadratic_formula(double a, double b, double c);

    };
}
