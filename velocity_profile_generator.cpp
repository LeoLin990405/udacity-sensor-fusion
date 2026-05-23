/*
 * TODO solutions for velocity_profile_generator.cpp.
 */

#include <algorithm>
#include <cmath>

double calc_distance_for_constant_accel(double initial_speed,
                                        double final_speed,
                                        double acceleration) {
  const double a = std::max(0.01, std::fabs(acceleration));
  return std::max(0.0, (final_speed * final_speed - initial_speed * initial_speed) /
                           (2.0 * acceleration));
}

double calc_final_speed(double initial_speed,
                        double acceleration,
                        double distance,
                        double speed_limit) {
  const double speed_squared =
      initial_speed * initial_speed + 2.0 * acceleration * distance;
  return std::min(speed_limit, std::sqrt(std::max(0.0, speed_squared)));
}
