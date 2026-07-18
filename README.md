# SFND 2D Feature Tracking

This project builds a camera feature-tracking pipeline for the preceding vehicle in the KITTI image sequence. It compares detector and descriptor combinations using keypoint coverage, retained matches, and runtime.

<img src="images/keypoints.png" width="820" height="248" alt="Matched keypoints on the preceding vehicle" />

## Rubric Implementation

### MP.0 Mid-Term Report

This README documents every rubric item, the experiment method, all valid detector/descriptor combinations, and the final recommendations. Reproducible raw results are available in [the combination summary](results/feature_tracking_benchmark.csv) and [the per-frame measurements](results/feature_tracking_frames.csv).

### MP.1 Data Buffer Optimization

`dataBuffer` stores `DataFrame` objects and is limited to `dataBufferSize = 2`. Before a new frame is appended, the oldest frame is erased whenever the limit has been reached. Only the current and previous images remain in memory.

### MP.2 Keypoint Detection

The detector is selectable at runtime. The implementation supports `SHITOMASI`, `HARRIS`, `FAST`, `BRISK`, `ORB`, `AKAZE`, and `SIFT`. Harris includes response thresholding and non-maximum suppression.

### MP.3 Keypoint Removal

Only keypoints inside `cv::Rect(535, 180, 180, 150)` are retained. This rectangle covers the preceding vehicle, so background features do not enter descriptor extraction or matching.

### MP.4 Keypoint Descriptors

The descriptor is selectable at runtime. The implementation supports `BRISK`, `BRIEF`, `ORB`, `FREAK`, `AKAZE`, and `SIFT`. SIFT uses the modern `cv::SIFT` API while BRIEF and FREAK use OpenCV contrib.

### MP.5 Descriptor Matching

Both brute-force (`MAT_BF`) and FLANN (`MAT_FLANN`) matching are implemented. Brute-force matching selects Hamming distance for binary descriptors and L2 distance for the SIFT floating-point descriptor. The benchmark uses brute-force matching as required by the rubric.

### MP.6 Descriptor Distance Ratio

K-nearest-neighbor matching uses `k = 2`. A match is retained only when the closest distance is less than `0.8` times the second-closest distance.

## Performance Evaluation

The benchmark processes all 10 KITTI images, keeps keypoints only in the vehicle ROI, and matches each frame to its predecessor using BF/KNN and the `0.8` ratio threshold. It ran in Release mode with OpenCV 4.4.0 on two identical Lenovo M720q workers, each with an Intel Core i7-8700T (6 cores, 12 threads). Times below are measured averages over the 10-image sequence.

`AKAZE` descriptors are valid only with the `AKAZE` detector. `SIFT + ORB` is excluded because that combination is incompatible with the tested OpenCV build.

### MP.7 Keypoint Counts and Neighborhood Size

The table reports keypoints retained in the vehicle ROI for every image. Mean size is the average keypoint neighborhood diameter across the sequence.

| Detector | Img 0 | Img 1 | Img 2 | Img 3 | Img 4 | Img 5 | Img 6 | Img 7 | Img 8 | Img 9 | Mean size (px) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| SHITOMASI | 125 | 118 | 123 | 120 | 120 | 113 | 114 | 123 | 111 | 112 | 4.00 |
| HARRIS | 17 | 14 | 18 | 21 | 26 | 42 | 18 | 31 | 26 | 34 | 6.00 |
| FAST | 419 | 427 | 404 | 423 | 386 | 414 | 418 | 406 | 396 | 401 | 7.00 |
| BRISK | 264 | 282 | 282 | 277 | 297 | 279 | 289 | 272 | 266 | 254 | 21.94 |
| ORB | 92 | 102 | 106 | 113 | 109 | 125 | 130 | 129 | 127 | 128 | 56.06 |
| AKAZE | 166 | 157 | 161 | 155 | 163 | 164 | 173 | 175 | 177 | 179 | 7.69 |
| SIFT | 138 | 132 | 124 | 137 | 134 | 140 | 137 | 148 | 159 | 137 | 5.03 |

FAST produces the densest and most consistent coverage in the ROI. Harris retains too few points for robust tracking. BRISK and ORB use much larger multi-scale neighborhoods than the other detectors.

### MP.8 Matched Keypoints

Each value is the mean number of accepted matches across the nine frame transitions. A dash marks an invalid combination.

| Detector | BRISK | BRIEF | ORB | FREAK | AKAZE | SIFT |
|---|---:|---:|---:|---:|---:|---:|
| SHITOMASI | 85.222 | 104.889 | 100.778 | 85.111 | - | 103.000 |
| HARRIS | 15.778 | 19.333 | 18.222 | 16.111 | - | 18.222 |
| FAST | 242.556 | 314.556 | 306.889 | 247.222 | - | 309.111 |
| BRISK | 174.444 | 189.333 | 167.778 | 169.556 | - | 182.889 |
| ORB | 83.444 | 60.556 | 84.556 | 46.778 | - | 84.778 |
| AKAZE | 135.000 | 140.667 | 131.778 | 132.000 | 139.889 | 141.111 |
| SIFT | 65.778 | 78.000 | - | 66.222 | - | 88.889 |

FAST paired with BRIEF, SIFT, or ORB retains the most matches. The Harris combinations consistently retain the fewest because the detector supplies only about 25 ROI keypoints per frame.

### MP.9 Detection and Description Time

Times are milliseconds per frame. Combined time is detector plus descriptor extraction time; matching time is intentionally excluded from this rubric measurement.

| Detector | Descriptor | Detect (ms/frame) | Describe (ms/frame) | Combined (ms/frame) | Mean matches |
|---|---|---:|---:|---:|---:|
| SHITOMASI | BRISK | 12.485 | 169.318 | 181.803 | 85.222 |
| SHITOMASI | BRIEF | 12.206 | 1.003 | 13.209 | 104.889 |
| SHITOMASI | ORB | 12.835 | 2.841 | 15.676 | 100.778 |
| SHITOMASI | FREAK | 9.297 | 21.733 | 31.030 | 85.111 |
| SHITOMASI | SIFT | 8.974 | 9.601 | 18.575 | 103.000 |
| HARRIS | BRISK | 11.752 | 191.639 | 203.391 | 15.778 |
| HARRIS | BRIEF | 11.501 | 0.752 | 12.252 | 19.333 |
| HARRIS | ORB | 11.381 | 2.592 | 13.973 | 18.222 |
| HARRIS | FREAK | 8.054 | 20.806 | 28.860 | 16.111 |
| HARRIS | SIFT | 12.402 | 14.472 | 26.873 | 18.222 |
| FAST | BRISK | 2.866 | 196.644 | 199.511 | 242.556 |
| FAST | BRIEF | 2.076 | 1.391 | 3.467 | 314.556 |
| FAST | ORB | 2.046 | 3.084 | 5.130 | 306.889 |
| FAST | FREAK | 5.641 | 45.851 | 51.491 | 247.222 |
| FAST | SIFT | 5.416 | 35.268 | 40.684 | 309.111 |
| BRISK | BRISK | 207.126 | 168.265 | 375.391 | 174.444 |
| BRISK | BRIEF | 206.595 | 0.945 | 207.539 | 189.333 |
| BRISK | ORB | 204.881 | 10.094 | 214.975 | 167.778 |
| BRISK | FREAK | 226.764 | 22.741 | 249.505 | 169.556 |
| BRISK | SIFT | 214.798 | 21.268 | 236.066 | 182.889 |
| ORB | BRISK | 6.574 | 169.540 | 176.114 | 83.444 |
| ORB | BRIEF | 6.213 | 0.408 | 6.621 | 60.556 |
| ORB | ORB | 10.084 | 19.020 | 29.104 | 84.556 |
| ORB | FREAK | 6.646 | 22.294 | 28.940 | 46.778 |
| ORB | SIFT | 13.884 | 36.612 | 50.495 | 84.778 |
| AKAZE | BRISK | 76.347 | 171.839 | 248.185 | 135.000 |
| AKAZE | BRIEF | 122.364 | 1.526 | 123.890 | 140.667 |
| AKAZE | ORB | 103.120 | 13.776 | 116.896 | 131.778 |
| AKAZE | FREAK | 107.505 | 41.186 | 148.691 | 132.000 |
| AKAZE | AKAZE | 113.125 | 94.940 | 208.064 | 139.889 |
| AKAZE | SIFT | 105.220 | 33.947 | 139.167 | 141.111 |
| SIFT | BRISK | 134.417 | 203.706 | 338.123 | 65.778 |
| SIFT | BRIEF | 115.594 | 2.238 | 117.832 | 78.000 |
| SIFT | FREAK | 127.442 | 48.764 | 176.207 | 66.222 |
| SIFT | SIFT | 128.443 | 84.844 | 213.287 | 88.889 |

## Top Three Recommendations

1. **FAST + BRIEF**: 3.467 ms/frame and 314.556 mean matches. It is both the fastest valid combination and the strongest by retained-match count.
2. **FAST + ORB**: 5.130 ms/frame and 306.889 mean matches. It is only slightly slower and provides similarly dense tracking with a rotation-aware binary descriptor.
3. **ORB + BRIEF**: 6.621 ms/frame and 60.556 mean matches. It is the third-fastest combination, but its lower match count makes it better suited to workloads where latency matters more than track density.

For this vehicle-tracking sequence, `FAST + BRIEF` is the best overall choice. `SHITOMASI + BRIEF` is a useful alternative when moderate feature density is preferred over FAST's much larger keypoint set.

## Build and Reproduce

Requirements are CMake, a C++11 compiler, and OpenCV 4.1 or newer built with the contrib `xfeatures2d` module.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
./build/2D_feature_tracking FAST BRIEF "$PWD/"
```

The executable accepts `DETECTOR`, `DESCRIPTOR`, an optional data-root path, and optional `--visualize`. It emits `METRIC` rows for each image and one `SUMMARY` row for the complete sequence.

Run every compatible combination and regenerate both CSV files with:

```bash
./scripts/run_benchmark.sh ./build/2D_feature_tracking "$PWD/" "$PWD/results"
```

Cluster workers can split the detector list through the `DETECTORS` environment variable. `scripts/merge_benchmarks.py` merges worker outputs deterministically.
