# Scan Matching Localization

This is a complete implementation of the Udacity Self-Driving Car Engineer
Scan Matching Localization project.

## Approach

Each accumulated CARLA LiDAR scan is downsampled with a 0.5-meter PCL voxel
grid. Normal Distributions Transform (NDT) then aligns the filtered scan to the
provided map, using the previous LiDAR-only estimate as its initial guess. The
accepted NDT transform updates the estimated vehicle pose and transforms the
red scan into the map frame for visualization.

The implementation uses the CARLA ground-truth transform only to draw the red
reference vehicle and calculate the rubric's error metric. Localization starts
at the known origin and, after that initialization, uses LiDAR scan matching
only. Implausibly large frame-to-frame registration jumps are rejected so a
single poor solve cannot corrupt subsequent initial guesses.

## Build and Run

From the Udacity project workspace:

```bash
./make-libcarla-install.sh
cmake -S . -B build
cmake --build build -j2
./run_carla.sh
```

In a second terminal, from this directory:

```bash
./build/cloud_loc
```

Click the PCL viewer, press the up arrow three times for medium speed, and use
the left and right arrows to steer. The run passes after at least 170 meters if
the maximum displayed position error remains at or below 1.2 meters.

## Offline Registration Test

The optional test creates local LiDAR scans from `map.pcd`, adds deterministic
measurement noise, and checks continuous localization plus a pose near the
170-meter evaluation region:

```bash
cmake -S . -B build-test -DBUILD_SCAN_MATCHER_TESTS=ON
cmake --build build-test --target scan_matcher_test -j2
cd build-test
ctest --output-on-failure
```

## Cluster Verification

The project was validated on the M720q-B x86_64 node in an isolated Ubuntu
20.04 container with GCC 9.4, PCL 1.10, and the official Udacity CARLA client
libraries:

- The complete `cloud_loc` target compiled and linked successfully.
- All 12 offline registration checks passed.
- Continuous-motion position errors stayed between 0.009 and 0.077 meters.
- The 170-meter-region position error was 0.069 meters.
- The executable loaded successfully and reached the CARLA connection step.

This revision also replaces the incomplete earlier submission with the complete
starter integration and reads CARLA LiDAR locations through the valid
`detection.x`, `detection.y`, and `detection.z` fields.
