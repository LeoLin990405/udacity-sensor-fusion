/*
 * TODO solutions for cost_functions.cpp.
 */

#include <cmath>
#include <limits>
#include <vector>

double euclidean_distance_3d(double ax, double ay, double az,
                             double bx, double by, double bz) {
  const double dx = ax - bx;
  const double dy = ay - by;
  const double dz = az - bz;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Circle place_collision_circle(const PathPoint& point, double circle_offset) {
  Circle circle;
  circle.x = point.x + circle_offset * std::cos(point.theta);
  circle.y = point.y + circle_offset * std::sin(point.theta);
  circle.z = point.z;
  return circle;
}

bool collides_with_actor(const Circle& circle,
                         const Actor& actor,
                         double circle_radius,
                         double actor_radius) {
  const double distance = euclidean_distance_3d(
      circle.x, circle.y, circle.z,
      actor.location.x, actor.location.y, actor.location.z);
  return distance <= circle_radius + actor_radius;
}

double goal_distance_cost(const std::vector<PathPoint>& spiral,
                          const Pose& main_goal) {
  if (spiral.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  const PathPoint& last = spiral.back();
  return euclidean_distance_3d(last.x, last.y, last.z,
                               main_goal.location.x,
                               main_goal.location.y,
                               main_goal.location.z);
}
