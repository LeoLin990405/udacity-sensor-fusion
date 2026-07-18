#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "behavior_planner_FSM.h"
#include "cost_functions.h"
#include "motion_planner.h"
#include "planning_params.h"
#include "velocity_profile_generator.h"

namespace {

constexpr double kTolerance = 1e-5;

bool close(double actual, double expected) {
  return std::fabs(actual - expected) <= kTolerance;
}

State make_state(float x, float y, float yaw = 0.0F) {
  State state{};
  state.location = {x, y, 0.0F};
  state.rotation = {0.0F, yaw, 0.0F};
  return state;
}

void test_behavior_planner() {
  BehaviorPlannerFSM planner(1.5, 8.0, 20.0, 3.0, 0.02, 0.0, 0.25,
                             1.5, 0.5);
  State ego = make_state(0.0F, 0.0F);
  ego.velocity = {6.0F, 0.0F, 0.0F};

  assert(close(planner.get_look_ahead_distance(ego), 12.0));

  State stop_goal = make_state(10.0F, 0.0F);
  bool is_junction = true;
  stop_goal = planner.state_transition(ego, stop_goal, is_junction, "none");
  assert(planner.get_active_maneuver() == DECEL_TO_STOP);
  assert(close(stop_goal.location.x, 9.5));
  assert(close(utils::magnitude(stop_goal.velocity), 0.0));

  ego.location = stop_goal.location;
  is_junction = false;
  const State held_goal =
      planner.state_transition(ego, make_state(50.0F, 50.0F), is_junction,
                               "none");
  assert(planner.get_active_maneuver() == STOPPED);
  assert(close(held_goal.location.x, stop_goal.location.x));
  assert(close(held_goal.location.y, stop_goal.location.y));

  planner.state_transition(ego, make_state(100.0F, 100.0F), is_junction,
                           "Green");
  assert(planner.get_active_maneuver() == FOLLOW_LANE);

  BehaviorPlannerFSM nominal_planner(1.5, 8.0, 20.0, 3.0, 0.02, 1.0, 0.25,
                                     1.5, 0.5);
  State lane_goal = make_state(4.0F, 5.0F, static_cast<float>(M_PI / 2.0));
  lane_goal =
      nominal_planner.state_transition(ego, lane_goal, is_junction, "none");
  assert(close(lane_goal.velocity.x, 0.0));
  assert(close(lane_goal.velocity.y, 3.0));
}

void test_offset_goals() {
  MotionPlanner planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);
  const State main_goal = make_state(10.0F, 20.0F, 0.0F);
  const auto goals = planner.generate_offset_goals(main_goal);

  assert(goals.size() == P_NUM_PATHS);
  assert(close(goals.front().location.x, 10.0));
  assert(close(goals.front().location.y, 17.0));
  assert(close(goals[P_NUM_PATHS / 2].location.y, 20.0));
  assert(close(goals.back().location.y, 23.0));
}

void test_cost_functions() {
  PathPoint point{};
  point.theta = 0.0;
  const std::vector<PathPoint> spiral{point};

  const State colliding_obstacle = make_state(0.0F, 0.0F);
  assert(std::isinf(cost_functions::collision_circles_cost_spiral(
      spiral, {colliding_obstacle})));

  const State distant_obstacle = make_state(100.0F, 100.0F);
  assert(close(cost_functions::collision_circles_cost_spiral(
                   spiral, {distant_obstacle}),
               0.0));

  PathPoint endpoint{};
  endpoint.x = 3.0;
  endpoint.y = 4.0;
  const State goal = make_state(0.0F, 0.0F);
  assert(close(cost_functions::close_to_main_goal_cost_spiral({endpoint}, goal),
               utils::logistic(5.0)));
  assert(std::isinf(
      cost_functions::close_to_main_goal_cost_spiral({}, goal)));
}

void test_velocity_equations() {
  VelocityProfileGenerator generator;
  assert(close(generator.calc_distance(10.0, 0.0, -2.0), 25.0));
  assert(std::isinf(generator.calc_distance(10.0, 0.0, 0.0)));
  assert(close(generator.calc_final_speed(0.0, 2.0, 9.0), 6.0));
  assert(close(generator.calc_final_speed(2.0, -3.0, 2.0), 0.0));
  assert(std::isinf(generator.calc_final_speed(
      0.0, 1.0, std::numeric_limits<double>::quiet_NaN())));
}

}  // namespace

int main() {
  static_assert(P_NUM_PATHS >= 5 && P_NUM_PATHS <= 7,
                "Path count should stay within the rubric's practical range");
  static_assert(P_NUM_PATHS % 2 == 1,
                "An odd path count is required for a centerline goal");
  static_assert(P_NUM_POINTS_IN_SPIRAL > 2,
                "Spirals need enough samples for collision checking");

  test_behavior_planner();
  test_offset_goals();
  test_cost_functions();
  test_velocity_equations();
  std::cout << "All motion-planning core tests passed." << std::endl;
  return 0;
}
