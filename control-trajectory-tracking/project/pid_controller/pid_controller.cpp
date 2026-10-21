/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr double kDefaultDeltaTime = 0.05;
constexpr double kMinimumDeltaTime = 1e-3;
constexpr double kMaximumDeltaTime = 1.0;
}  // namespace

PID::PID()
    : p_error_(0.0),
      i_error_(0.0),
      d_error_(0.0),
      previous_error_(0.0),
      kp_(0.0),
      ki_(0.0),
      kd_(0.0),
      output_limit_max_(1.0),
      output_limit_min_(-1.0),
      delta_time_(kDefaultDeltaTime),
      has_previous_error_(false) {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
    kp_ = Kpi;
    ki_ = Kii;
    kd_ = Kdi;
    output_limit_max_ = std::max(output_lim_maxi, output_lim_mini);
    output_limit_min_ = std::min(output_lim_maxi, output_lim_mini);

    p_error_ = 0.0;
    i_error_ = 0.0;
    d_error_ = 0.0;
    previous_error_ = 0.0;
    delta_time_ = kDefaultDeltaTime;
    has_previous_error_ = false;
}

void PID::UpdateError(double cte) {
    p_error_ = cte;
    d_error_ = has_previous_error_
                   ? (cte - previous_error_) / delta_time_
                   : 0.0;
    i_error_ += cte * delta_time_;

    // Limit the integral contribution to the command range. This prevents a
    // prolonged saturation from causing a large overshoot when the error flips.
    if (std::abs(ki_) > 1e-12) {
        const double command_limit =
            std::max(std::abs(output_limit_min_), std::abs(output_limit_max_));
        const double integral_limit = command_limit / std::abs(ki_);
        i_error_ = std::max(-integral_limit,
                            std::min(i_error_, integral_limit));
    }

    previous_error_ = cte;
    has_previous_error_ = true;
}

double PID::TotalError() {
    const double control =
        kp_ * p_error_ + ki_ * i_error_ + kd_ * d_error_;
    return std::max(output_limit_min_,
                    std::min(control, output_limit_max_));
}

double PID::UpdateDeltaTime(double new_delta_time) {
    if (std::isfinite(new_delta_time) && new_delta_time > 0.0) {
        delta_time_ = std::max(kMinimumDeltaTime,
                               std::min(new_delta_time, kMaximumDeltaTime));
    }
    return delta_time_;
}
