#include "control_errors.h"

#include <cmath>

namespace control {
double NormalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

double ComputeSteeringError(double x_position, double y_position,
                            double current_yaw,
                            const std::vector<double>& trajectory_x,
                            const std::vector<double>& trajectory_y) {
    if (trajectory_x.empty() || trajectory_y.empty()) {
        return 0.0;
    }

    const auto target_index =
        std::min(trajectory_x.size(), trajectory_y.size()) - 1;
    const double target_dx = trajectory_x[target_index] - x_position;
    const double target_dy = trajectory_y[target_index] - y_position;
    if (std::hypot(target_dx, target_dy) < 1e-6) {
        return 0.0;
    }

    const double desired_yaw = std::atan2(target_dy, target_dx);
    return NormalizeAngle(desired_yaw - current_yaw);
}

double ComputeThrottleError(double current_velocity,
                            const std::vector<double>& trajectory_velocity) {
    if (trajectory_velocity.empty()) {
        return 0.0;
    }
    return trajectory_velocity.back() - current_velocity;
}

}  // namespace control
