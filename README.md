# Scan Matching Localization

This submission contains the implementation for Udacity Self Driving Car Engineer Course 4, Project 6: Scan Matching Localization.

## Approach

The solution follows the three required project steps:

1. Filter the incoming lidar scan with a PCL voxel grid filter.
2. Estimate the ego pose by matching the filtered scan against `map.pcd`.
3. Transform the filtered scan into the map frame and render the aligned scan.

The main registration method is Normal Distributions Transform (NDT), initialized from the previous estimated pose.  If NDT does not converge cleanly or returns a high fitness score, the code runs a short Iterative Closest Point (ICP) refinement.  This keeps the localization stable while the vehicle is moving at the required medium speed.

## Submitted File

- `c3-main.cpp` contains the helper functions and TODO replacement block for the Udacity workspace starter file.

## Workspace Usage

In the Udacity workspace:

```bash
cd /home/workspace/c3-project
cmake .
make
cd /home/workspace
./cloud_loc
```

Then launch CARLA with `./run-carla.sh` in the second terminal tab, click inside the PCL viewer, and drive with three up-arrow taps.  The algorithm uses ground truth only for the starter's initial pose and localizes from lidar scan matching afterward.
