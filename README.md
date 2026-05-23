# Control and Trajectory Tracking for Autonomous Vehicles

This repository contains the PID controller implementation, simulator-facing error definitions, generated PID logs, plots, and the required analysis report.

## Files

- `pid_controller.h` and `pid_controller.cpp` implement the PID class.
- `main.cpp` demonstrates the throttle and steering error calculations used in the simulator TODOs.
- `report.md` answers the project analysis questions and includes the generated plots.
- `plots/` contains the throttle and steering PID plots.

## Build

```bash
mkdir -p build
cd build
cmake ..
make
./pid_demo
```

The demo writes `throttle_data.txt` and `steer_data.txt`, matching the data columns used for the report plots.
