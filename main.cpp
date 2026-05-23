#include "pid_controller.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

double NormalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

int main() {
  PIDController throttle_pid(0.35, 0.004, 0.08, -1.0, 1.0);
  PIDController steer_pid(0.85, 0.0005, 0.18, -1.2, 1.2);

  std::ofstream throttle_log("throttle_data.txt");
  std::ofstream steer_log("steer_data.txt");
  throttle_log << "step,error,command\n";
  steer_log << "step,error,command\n";

  double velocity = 0.0;
  double yaw = 0.0;
  double x_position = 0.0;
  double y_position = 0.0;
  const double dt = 0.05;

  for (int step = 0; step < 300; ++step) {
    const double desired_velocity = 8.0 + 1.5 * std::sin(step * 0.04);
    const double target_x = x_position + 8.0;
    const double target_y = 1.2 * std::sin(step * 0.03);

    // Step 2: throttle error is desired speed minus current speed.  Positive
    // error means the car is too slow and the PID should command throttle.
    const double error_throttle = desired_velocity - velocity;
    throttle_pid.UpdateError(error_throttle, dt);
    const double throttle_cmd = throttle_pid.TotalError();

    // Step 3: steering error is the normalized angular difference between the
    // current yaw and the heading from the ego pose to the planned trajectory
    // point.  This directly controls cross-track heading correction.
    const double desired_yaw =
        std::atan2(target_y - y_position, target_x - x_position);
    const double error_steer = NormalizeAngle(desired_yaw - yaw);
    steer_pid.UpdateError(error_steer, dt);
    const double steer_cmd = steer_pid.TotalError();

    velocity = std::max(0.0, velocity + throttle_cmd * dt);
    yaw = NormalizeAngle(yaw + steer_cmd * dt * 0.2);
    x_position += velocity * std::cos(yaw) * dt;
    y_position += velocity * std::sin(yaw) * dt;

    throttle_log << step << "," << error_throttle << "," << throttle_cmd << "\n";
    steer_log << step << "," << error_steer << "," << steer_cmd << "\n";
  }

  std::cout << "Generated throttle_data.txt and steer_data.txt" << std::endl;
  return 0;
}
