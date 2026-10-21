#ifndef CONTROL_ERRORS_H
#define CONTROL_ERRORS_H

#include <vector>

namespace control {

double NormalizeAngle(double angle);

double ComputeSteeringError(double x_position, double y_position,
                            double current_yaw,
                            const std::vector<double>& trajectory_x,
                            const std::vector<double>& trajectory_y);

double ComputeThrottleError(double current_velocity,
                            const std::vector<double>& trajectory_velocity);

}  // namespace control

#endif  // CONTROL_ERRORS_H
