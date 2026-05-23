/*
 * TODO solutions for motion_planner.cpp.
 *
 * Candidate goals are generated on a line perpendicular to the centerline goal.
 * With an odd P_NUM_PATHS, i == P_NUM_PATHS / 2 is exactly the center goal.
 */

#include <cmath>
#include <vector>

std::vector<Pose> generate_offset_goals(const Pose& main_goal,
                                        double goal_offset,
                                        int num_paths) {
  std::vector<Pose> goals;
  goals.reserve(num_paths);

  const double yaw_plus_90 = main_goal.rotation.yaw + M_PI / 2.0;
  const int center_index = num_paths / 2;

  for (int i = 0; i < num_paths; ++i) {
    const double offset = (i - center_index) * goal_offset;
    Pose goal = main_goal;
    goal.location.x = main_goal.location.x + offset * std::cos(yaw_plus_90);
    goal.location.y = main_goal.location.y + offset * std::sin(yaw_plus_90);
    goals.push_back(goal);
  }

  return goals;
}
