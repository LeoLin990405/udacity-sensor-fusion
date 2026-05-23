# Motion Planning and Decision Making for Autonomous Vehicles

This submission provides the completed TODO logic requested by the Udacity project rubric.

## Completed Files

- `behavior_planner_FSM.cpp`
- `cost_functions.cpp`
- `motion_planner.cpp`
- `velocity_profile_generator.cpp`
- `planning_params.h`

## Implementation Summary

The behavior planner implements the required finite-state machine:

- `FOLLOW_LANE` computes a stopping-distance-based lookahead and drives nominally at the speed limit.
- `DECEL_TO_STOP` keeps the stop-line goal fixed and uses distance to decide when the ego vehicle has arrived.
- `STOPPED` holds the same stop goal for the required dwell period before returning to lane following when safe.

The motion planner generates seven laterally offset cubic-spiral goals around the centerline goal.  The velocity planner uses rectilinear constant-acceleration equations for deceleration and nominal speed profiles.  The cost functions place collision-checking circles along each spiral, compare actor distances against circle radii, and prefer collision-free spirals that end closest to the centerline goal.

## Workspace Notes

Copy the TODO implementations into the matching starter files in the Udacity workspace, then compile and run the project workspace as instructed in the classroom.
