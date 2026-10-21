# Validation

## Dependency-isolated control tests

Run the production PID and tracking-error sources with strict warnings:

```bash
./tests/run_core_tests.sh
```

The suite checks each P/I/D term, output saturation, safe time-step handling,
angle wrapping, throttle and steering error signs, and deterministic closed-loop
convergence.

Compile every CMake translation unit, including `main.cpp`, against lightweight
CARLA/uWS API declarations:

```bash
./tests/run_source_compile_check.sh
```

The declarations only isolate the external link boundary; all compiled project
sources are the production files shipped in the submission.

## CARLA integration boundary

The complete Udacity starter workspace and `CMakeLists.txt` are included under
`project/`. The full executable links against the course VM's fixed
`/opt/carla-source` and CARLA 0.9.9.4 installation. Those course-only libraries
are intentionally not vendored or installed on the validation hosts. In the
Udacity VM, build and launch with the commands documented in `README.md`.
