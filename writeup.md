# Writeup: Track 3D-Objects Over Time

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
