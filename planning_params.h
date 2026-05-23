#ifndef PLANNING_PARAMS_H
#define PLANNING_PARAMS_H

// Number of laterally offset candidate goals.  The value is odd so one spiral
// always targets the lane centerline while symmetric alternatives are generated
// on both sides.
constexpr int P_NUM_PATHS = 7;

// Number of samples along each cubic spiral.  This is dense enough for circle
// collision checks and smooth visualization without slowing the simulator.
constexpr int P_NUM_POINTS_IN_SPIRAL = 25;

constexpr double P_STOP_THRESHOLD_DISTANCE = 1.0;
constexpr double P_STOP_THRESHOLD_SPEED = 0.02;

#endif  // PLANNING_PARAMS_H
