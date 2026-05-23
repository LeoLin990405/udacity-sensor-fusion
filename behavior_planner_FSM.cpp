/*
 * TODO solutions for behavior_planner_FSM.cpp.
 *
 * The snippets below are written to match the Udacity starter logic:
 * FOLLOW_LANE -> DECEL_TO_STOP -> STOPPED -> FOLLOW_LANE.  They avoid using
 * speed as the stop trigger because the simulator teleports the ego vehicle at
 * zero speed; instead, the stop transition uses distance to the stop goal.
 */

#include <cmath>

double compute_lookahead_distance(double ego_speed, double comfortable_decel) {
  const double min_lookahead = 8.0;
  const double reaction_buffer = 6.0;
  const double decel = std::max(0.1, std::fabs(comfortable_decel));
  return std::max(min_lookahead, reaction_buffer + ego_speed * ego_speed / (2.0 * decel));
}

void set_stop_goal(Pose& goal, double stop_line_buffer) {
  const double ang = goal.rotation.yaw + M_PI;
  goal.location.x += stop_line_buffer * std::cos(ang);
  goal.location.y += stop_line_buffer * std::sin(ang);
  goal.velocity.x = 0.0;
  goal.velocity.y = 0.0;
  goal.velocity.z = 0.0;
}

void set_nominal_goal_speed(Pose& goal, double speed_limit) {
  goal.velocity.x = speed_limit * std::cos(goal.rotation.yaw);
  goal.velocity.y = speed_limit * std::sin(goal.rotation.yaw);
  goal.velocity.z = 0.0;
}

double distance_to_goal(const Pose& ego_pose, const Pose& goal) {
  const double dx = goal.location.x - ego_pose.location.x;
  const double dy = goal.location.y - ego_pose.location.y;
  const double dz = goal.location.z - ego_pose.location.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/*
 * State update policy to paste into the starter transition logic:
 *
 * FOLLOW_LANE:
 *   - Generate the lane-center goal at compute_lookahead_distance().
 *   - If the upcoming stop line is inside the lookahead region, call
 *     set_stop_goal(goal, _stop_line_buffer), save _goal = goal, and set
 *     _state = DECEL_TO_STOP.
 *   - Otherwise call set_nominal_goal_speed(goal, _speed_limit).
 *
 * DECEL_TO_STOP:
 *   - Keep goal = _goal so the stop line target does not move.
 *   - If distance_to_goal(ego_pose, _goal) <= P_STOP_THRESHOLD_DISTANCE,
 *     transition to STOPPED and start/reset the stop timer.
 *
 * STOPPED:
 *   - Keep goal = _goal and velocity zero.
 *   - Once the five-second stop requirement is satisfied and the intersection
 *     is safe, transition back to FOLLOW_LANE.
 */
