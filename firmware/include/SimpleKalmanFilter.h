#pragma once

#include <Arduino.h>

#include <math.h>

class SimpleKalmanFilter {
 public:
  SimpleKalmanFilter(float error_measure, float error_estimate, float process_noise)
      : error_measure_(error_measure),
        error_estimate_(error_estimate),
        process_noise_(process_noise),
        current_estimate_(0.0f),
        last_estimate_(0.0f),
        kalman_gain_(0.0f) {}

  float updateEstimate(float measurement) {
    kalman_gain_ = error_estimate_ / (error_estimate_ + error_measure_);
    current_estimate_ = last_estimate_ + (kalman_gain_ * (measurement - last_estimate_));
    error_estimate_ =
        (1.0f - kalman_gain_) * error_estimate_ +
        fabsf(last_estimate_ - current_estimate_) * process_noise_;
    last_estimate_ = current_estimate_;
    return current_estimate_;
  }

 private:
  float error_measure_;
  float error_estimate_;
  float process_noise_;
  float current_estimate_;
  float last_estimate_;
  float kalman_gain_;
};
