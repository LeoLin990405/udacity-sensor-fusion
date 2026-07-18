# Review Response

The ungradeable review for submission 4778660 identified a coordinate-frame
compatibility issue in the CARLA LiDAR callback. The callback now follows the
reviewer-provided starter revision and converts each accepted detection from
the current Udacity CARLA sensor frame to the PCL/course map frame:

```cpp
PointT(-detection.y, detection.x, -detection.z)
```

The complete NDT localization implementation remains in place. A regression
check in `tests/lidar_coordinate_regression.sh` verifies that the required
conversion is present in the submitted integration source.
