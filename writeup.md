# Writeup: Track 3D-Objects Over Time

## Mid-Term Project: 3D Object Detection

### Lidar Point Cloud and Range Image

The top lidar was selected because it provides the most complete 360-degree field of view and the densest point cloud among the available sensors. I implemented point-cloud visualization with Open3D and range-image generation by extracting the range and intensity channels from the Waymo lidar frame. Negative range and intensity values are removed, range is scaled into an 8-bit image, and intensity is clipped by percentile before normalization so that highly reflective outliers do not dominate the visualization.

### Birds-Eye View Preprocessing

The point cloud is cropped to the configured detection limits, shifted in height so the lowest configured z value maps to zero, and discretized into BEV coordinates. The intensity map keeps the top-most lidar point per BEV cell and normalizes reflectivity. The height map records the normalized z value of the top point, and the density map uses log-normalized point counts. These three maps are stacked into a tensor with the expected channel order for the object detector.

### Detection and Post-Processing

The implementation supports both Darknet/Complex YOLO and FPN ResNet model configurations. The FPN ResNet configuration defines its heads for center heatmap, center offset, direction, z coordinate, and dimensions, then decodes and post-processes model outputs. Final detections are converted from BEV pixel coordinates into metric vehicle coordinates using the configured x and y limits, BEV dimensions, and discretization factor. The returned object format contains class id, center position, box dimensions, and yaw angle.

### Evaluation

Detection performance is evaluated by matching valid ground-truth labels to detections using polygon IoU in BEV. For each valid label, the best detection above the IoU threshold is selected, and the x, y, and z center deviations are recorded. Precision is computed from true positives and false positives, while recall is computed from true positives and all valid positives. The evaluation code also supports plotting histograms for IoU and center deviations.

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?

The tracking pipeline follows four major stages. First, the Kalman filter predicts each track state with a constant-velocity motion model and a process-noise matrix based on the configured time step and acceleration noise. Second, measurements update the state with either the linear lidar model or the nonlinear camera projection model. Third, the association module computes the Mahalanobis distance between tracks and measurements, gates unlikely matches with the chi-square threshold, and greedily assigns the lowest valid distances. Fourth, the track-management module maintains tentative, initialized, and confirmed tracks, increases scores after successful updates, decreases scores when visible tracks miss detections, initializes new lidar-based tracks, and deletes unreliable or highly uncertain tracks.

I implemented the filter prediction and update equations, lidar and camera measurement handling, camera field-of-view checks, Mahalanobis-distance association, gating, track score management, and the object-detection preprocessing and evaluation functions. The completed pipeline is designed to keep stable confirmed tracks for vehicles while reducing short-lived false tracks from noisy detections. The most difficult part was the camera fusion path because the state must be transformed into camera coordinates before projection, and the projection model is nonlinear and sensitive to incorrect coordinate-frame assumptions.

### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)?

Yes. Lidar provides strong metric depth and object size cues, but camera measurements can help maintain or refine tracks when lidar detections are sparse, partially occluded, or temporarily missed. In theory, fusing both sensors improves robustness because the sensors fail in different ways: lidar can struggle with long range and sparse returns, while cameras can provide dense angular observations but weaker depth information.

In this implementation, camera updates are only applied after passing the camera field-of-view test and Mahalanobis gating. That keeps camera detections from corrupting tracks outside the image or from unrelated objects. Compared with a lidar-only update sequence, this fusion strategy gives the filter additional measurement opportunities and can reduce uncertainty for visible vehicles, especially in image-plane position.

### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?

Real sensor-fusion systems face calibration errors, timestamp misalignment, occlusions, missed detections, false positives, sensor range differences, adverse weather, and changing object appearances. Fusion also depends heavily on robust data association; a single wrong association can pull a track toward the wrong object and make later updates worse.

The project exposes several of those issues in a controlled way. The lidar and camera measurements use different dimensions and measurement models, so the filter must use sensor-specific `H`, `hx`, and `R` values. Some detections are missing or noisy, so track scores and covariance-based deletion are necessary. The association matrix also needs gating because not every measurement should be eligible for every track.

### 4. Can you think of ways to improve your tracking results in the future?

The tracker could be improved with a more expressive motion model, such as constant-turn-rate or interacting multiple models for turning vehicles. Association could also be improved with a global optimizer, such as Hungarian assignment, instead of greedy closest-distance matching.

On the perception side, using a stronger detector, detector confidence calibration, and temporal smoothing of detections would reduce false positives and false negatives before tracking. A production system would also benefit from explicit timestamp synchronization, online calibration checks, richer object classification, and quantitative evaluation over a larger set of Waymo sequences.
