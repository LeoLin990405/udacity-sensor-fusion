# Motion Planning and Decision Making for Autonomous Vehicles

This submission is based on Udacity's complete
[`nd013-c5-planning-starter`](https://github.com/udacity/nd013-c5-planning-starter)
workspace at commit `52d356ca4cbe0980637a020bb93ced9ea8cbd70a`. The previous
submission contained only five detached snippets and could not compile. This
version restores the full project structure and implements every required TODO
in the original classes.

## Rubric Implementation

- `behavior_planner_FSM.cpp`: braking-distance lookahead; buffered stop goal;
  nominal and stop velocities; stable `FOLLOW_LANE -> DECEL_TO_STOP -> STOPPED
  -> FOLLOW_LANE` transitions using distance to the fixed stop goal.
- `planning_params.h`: seven symmetric candidate paths and 25 samples per
  cubic spiral.
- `motion_planner.cpp`: laterally offset goals on the line perpendicular to the
  road heading.
- `velocity_profile_generator.cpp`: constant-acceleration distance and final
  speed equations, including zero acceleration and invalid discriminants.
- `cost_functions.cpp`: oriented collision-circle placement, actor clearance,
  and distance from each spiral endpoint to the main goal.

## Project Layout

The complete runnable workspace is under `project/`. The files evaluated by the
rubric are in `project/starter_files/`, together with the required headers,
Eigen sources, CMake configuration, simulator bridge, and launch scripts.

## Build in the Course Environment

The official build expects the Udacity Ubuntu 20.04/CARLA 0.9.9.4 environment,
including `/opt/carla-source` and its prebuilt libraries:

```bash
cd project/starter_files
cmake .
make -j2
```

To run the integrated simulator after CARLA is started:

```bash
cd project
./run_main.sh
```

## Core Regression Tests

The dependency-isolated tests compile the submitted production sources with
small CARLA type stubs. They cover all rubric formulas and state transitions:

```bash
./tests/run_core_tests.sh
```

See `VALIDATION.md` for the exact validation evidence and `REVIEW_RESPONSE.md`
for the correction made after the previous ungradeable submission.
