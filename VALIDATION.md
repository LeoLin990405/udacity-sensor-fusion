# Validation

Validation is performed without installing system packages on any host.

## Core C++ Tests

Run from the repository root:

```bash
./tests/run_core_tests.sh
```

The test binary compiles the submitted production implementations for behavior
planning, motion planning, collision costs, cubic spirals, utilities, and
velocity profiles. CARLA stubs provide only the data types needed to compile;
all tested algorithms come from `project/starter_files/`.

The suite passed in both available project environments:

- Mac M4 (`arm64`, Apple Clang 21.0.0)
- M720q-A (`x86_64`, GCC 15.2.0)

Both runs reported:

```text
All motion-planning core tests passed.
```

The tests exercise the braking-distance lookahead, all behavior-planner state
transitions and target velocities, seven symmetric offset goals, collision
circle geometry, endpoint cost, spiral sample constants, and both constant-
acceleration equations. The source tree was also compared against official
starter commit `52d356ca4cbe0980637a020bb93ced9ea8cbd70a`; the only differences are the
rubric implementations and small standard-header/override fixes needed by the
two compilers.

## Full Course Build

The complete official CMake project is retained under `project/starter_files`.
It targets Udacity's CARLA 0.9.9.4 environment at `/opt/carla-source`, exactly
as described by the current course starter.

The mini hosts do not contain that course-only CARLA source/build tree, so the
simulator executable was not linked there. No system packages were installed
on either host. The complete source and CMake layout are included so the
reviewer can run the normal `cmake . && make -j2` command in Udacity's supplied
course environment.
