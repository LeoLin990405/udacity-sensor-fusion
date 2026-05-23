#include "pid_controller.h"

#include <algorithm>

PIDController::PIDController(double kp, double ki, double kd, double min_output,
                             double max_output)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      min_output_(min_output),
      max_output_(max_output),
      p_error_(0.0),
      i_error_(0.0),
      d_error_(0.0),
      previous_error_(0.0),
      has_previous_error_(false) {}

void PIDController::UpdateError(double error, double dt) {
  const double safe_dt = std::max(dt, 1e-3);

  p_error_ = error;
  i_error_ += error * safe_dt;
  d_error_ = has_previous_error_ ? (error - previous_error_) / safe_dt : 0.0;

  previous_error_ = error;
  has_previous_error_ = true;
}

double PIDController::TotalError() const {
  const double raw_output = -(kp_ * p_error_ + ki_ * i_error_ + kd_ * d_error_);
  return std::clamp(raw_output, min_output_, max_output_);
}

void PIDController::Reset() {
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
  previous_error_ = 0.0;
  has_previous_error_ = false;
}
