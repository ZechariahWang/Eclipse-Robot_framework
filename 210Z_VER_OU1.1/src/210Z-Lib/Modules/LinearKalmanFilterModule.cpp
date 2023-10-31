/**
 * @file KFilter.cpp
 * @author Zechariah Wang
 * @brief Linear Kalman filter for better estimates of robot future metrics
 * @version 0.1
 * @date 2023-07-14
 */

#include "main.h"

using namespace Eclipse;

Eclipse::KalmanFilter::KalmanFilter(){
    kal.lateral_process_noise_covariance = 0.01;
    kal.lateral_measurement_noise_covariance = 0.1;
    kal.x_process_noise_covariance = 0.01;
    kal.x_measurement_noise_covariance = 0.1;
    kal.y_process_noise_covariance = 0.01;
    kal.y_measurement_noise_covariance = 0.1;
}

void Eclipse::KalmanFilter::update_lateral_components(){
    kal.lateral_state_estimate = 0;
    kal.lateral_error_covariance = 1;
}

double Eclipse::KalmanFilter::lateral_prediction_step(){
    kal.lateral_predicted_value = kal.lateral_state_estimate;
    kal.lateral_predicted_p = kal.lateral_error_covariance + kal.lateral_measurement_noise_covariance;
    return kal.lateral_predicted_p;
}

double Eclipse::KalmanFilter::lateral_update_filter_step(double raw_position){
    double residual = raw_position - kal.lateral_predicted_value;
    double gain = kal.lateral_predicted_p / (kal.lateral_predicted_p + kal.lateral_measurement_noise_covariance);
    kal.lateral_state_estimate = kal.lateral_predicted_value + gain * residual;
    kal.lateral_error_covariance = (1 - gain) * kal.lateral_predicted_p;
    return kal.lateral_state_estimate;
}

void Eclipse::KalmanFilter::update_x_components(){
    kal.x_state_estimate = 0;
    kal.x_error_covariance = 1;
}

double Eclipse::KalmanFilter::x_prediction_step(){
    kal.x_predicted_value = kal.x_state_estimate;
    kal.x_predicted_p = kal.x_error_covariance + kal.x_measurement_noise_covariance;
    return kal.x_predicted_p;
}

double Eclipse::KalmanFilter::x_update_filter_step(double raw_position){
    double residual = raw_position - kal.x_predicted_value;
    double gain = kal.x_predicted_p / (kal.x_predicted_p + kal.x_measurement_noise_covariance);
    kal.x_state_estimate = kal.x_predicted_value + gain * residual;
    kal.x_error_covariance = (1 - gain) * kal.x_predicted_p;
    return kal.x_state_estimate;
}

void Eclipse::KalmanFilter::update_y_components(){
    kal.y_state_estimate = 0;
    kal.y_error_covariance = 1;
}

double Eclipse::KalmanFilter::y_prediction_step(){
    kal.y_predicted_value = kal.y_state_estimate;
    kal.y_predicted_p = kal.y_error_covariance + kal.y_measurement_noise_covariance;
    return kal.y_predicted_p;
}

double Eclipse::KalmanFilter::y_update_filter_step(double raw_position){
    double residual = raw_position - kal.y_predicted_value;
    double gain = kal.y_predicted_p / (kal.y_predicted_p + kal.y_measurement_noise_covariance);
    kal.y_state_estimate = kal.y_predicted_value + gain * residual;
    kal.y_error_covariance = (1 - gain) * kal.y_predicted_p;
    return kal.y_state_estimate;
}