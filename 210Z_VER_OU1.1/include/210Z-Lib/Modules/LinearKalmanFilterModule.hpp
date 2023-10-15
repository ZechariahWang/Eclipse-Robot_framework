#include "iostream"

namespace Eclipse{
    class KalmanFilter{
        public:
            KalmanFilter();

            double lateral_state_estimate;
            double lateral_error_covariance;
            double lateral_process_noise_covariance;
            double lateral_measurement_noise_covariance;

            double lateral_predicted_value;
            double lateral_predicted_p;

            double lateral_prediction_step();
            double lateral_update_filter_step(double raw_position);
            void   update_lateral_components();

            double x_state_estimate;
            double x_error_covariance;
            double x_process_noise_covariance;
            double x_measurement_noise_covariance;

            double x_predicted_value;
            double x_predicted_p;

            double x_prediction_step();
            double x_update_filter_step(double raw_position);
            void   update_x_components();

            double y_state_estimate;
            double y_error_covariance;
            double y_process_noise_covariance;
            double y_measurement_noise_covariance;

            double y_predicted_value;
            double y_predicted_p;

            double y_prediction_step();
            double y_update_filter_step(double raw_position);
            void   update_y_components();

    };
}