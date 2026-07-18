# Control and Trajectory Tracking for Autonomous Vehicles

This project implements independent PID controllers for longitudinal speed and
lateral heading control in Udacity's CARLA trajectory-tracking workspace. It is
based on the complete official `nd013-c6-control-starter` at commit
`2b5da491f595738af35342eec3022ccb167b9877`.

## Implementation Summary

- `PID::UpdateError` computes the discrete proportional, integral, and
  derivative errors using the current simulator time step.
- `PID::TotalError` applies the tuned gains and clamps commands to the simulator
  limits.
- Integral anti-windup bounds the integral contribution during command
  saturation.
- Throttle error is the final planned speed minus measured vehicle speed.
- Steering error is the normalized angular difference between current yaw and
  the heading from the ego pose to the final planned trajectory point.
- Positive longitudinal commands become throttle; negative commands become
  brake.

The selected gains are:

| Controller | Kp | Ki | Kd | Output range |
| --- | ---: | ---: | ---: | ---: |
| Steering | 0.30 | 0.0010 | 0.30 | `[-1.2, 1.2]` |
| Throttle | 0.20 | 0.0009 | 0.10 | `[-1.0, 1.0]` |

## Project Layout

| Path | Purpose |
| --- | --- |
| `project/pid_controller/main.cpp` | CARLA websocket, planning, errors, and control commands |
| `project/pid_controller/pid_controller.*` | Discrete PID implementation |
| `project/pid_controller/control_errors.*` | Testable speed and heading error functions |
| `project/pid_controller/` | Complete planning and trajectory-tracking C++ workspace |
| `project/simulatorAPI.py` | Udacity CARLA simulator client |
| `project/run_main_pid.sh` | Launches the built control server and simulator client |
| `project/plot_pid.py` | Plots logs produced by the CARLA run |
| `tests/` | Strict compiler tests and deterministic response experiment |
| `report.md` | Results and rubric discussion |

## Build and Run in the Udacity VM

The integration target expects the course VM's CARLA 0.9.9.4 installation and
its fixed `/opt/carla-source` development tree.

Start CARLA in one terminal:

```bash
/opt/carla-simulator/CarlaUE4.sh
```

Build the C++ server in a second terminal:

```bash
cd project/pid_controller
rm -rf build
mkdir build
cd build
cmake ..
make -j2
cp pid_controller ..
```

Launch the controller and simulator client from `project/`:

```bash
cd ../..
./run_main_pid.sh
```

The run writes `steer_pid_data.txt` and `throttle_pid_data.txt`. Plot them from
the same directory with:

```bash
python3 plot_pid.py
```

## Dependency-Isolated Validation

The controller and error calculations can be validated without installing
CARLA or modifying a host system:

```bash
./tests/run_core_tests.sh
```

To regenerate the gain-comparison data and report plots, use a Python
environment with Pillow:

```bash
PYTHON_BIN=python3 ./tests/generate_response_artifacts.sh
```

See `VALIDATION.md` for the validation boundary and `REVIEW_RESPONSE.md` for the
specific correction made after the previous review.
