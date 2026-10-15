# Real-Time Visual SLAM

[![CI](https://github.com/Adiaharoni55/realtime-visual-slam/actions/workflows/ci.yml/badge.svg)](https://github.com/Adiaharoni55/realtime-visual-slam/actions/workflows/ci.yml)

A real-time C++17 SLAM system that estimates motion and builds a 3D map from a camera and a LiDAR. A deep-learning segmentation model (ONNX Runtime) masks moving objects such as people and cars, so they don't corrupt tracking.

> **Status: work in progress.** The foundations (build system, CI, camera geometry, dataset loading) are done and tested. Visual odometry is being built now.

## Progress

| Step | Status |
| --- | --- |
| 1. Project setup: CMake, sanitizers, unit tests, CI on Linux / macOS / Windows | ✅ Done |
| 2. Camera geometry: pinhole model, poses and quaternions, YAML camera config | ✅ Done |
| 3. Visual odometry on TUM RGB-D | 🚧 In progress (dataset loader done) |
| 4. Backend: bundle adjustment, pose graph, loop closure | Planned |
| 5. Masking moving objects with a deep-learning model (ONNX Runtime) | Planned |
| 6. LiDAR + camera fusion on KITTI | Planned |
| 7. Real-time performance: multithreading, profiling | Planned |
| 8. Docker, ROS 2 node, visualization | Planned |
| 9. Final results and evaluation | Planned |

## What works today

- **Pinhole camera model:** project 3D points to pixels and back-project pixels with depth to 3D, with input validation.
- **Pose utilities:** TUM-format poses (translation + quaternion) to 4×4 rigid transforms, and relative motion between poses.
- **Camera config:** intrinsics loaded from YAML files in `configs/`.
- **TUM RGB-D loader:** reads image lists and ground truth, and associates RGB, depth and ground-truth poses by timestamp (binary search, validated input).
- **Unit tests** for all of the above (GoogleTest), run in CI on every push.

## Planned pipeline

1. Read camera frames (TUM RGB-D, KITTI) and LiDAR scans (KITTI).
2. Segment moving objects with an ONNX model and build a mask.
3. Detect and match ORB features outside the mask; estimate motion with PnP + RANSAC.
4. Project LiDAR points into the image, drop those on moving objects, and align scans with ICP.
5. Optimize with a pose graph, bundle adjustment and loop closure.
6. Visualize with Rerun and score against ground truth (ATE / RPE).

## Tech stack

C++17 · CMake + presets · vcpkg · Eigen · OpenCV · yaml-cpp · GoogleTest · GitHub Actions · ASan / UBSan
*Planned:* Ceres, ONNX Runtime, Open3D, Rerun, Docker, ROS 2

## Build

**Requirements:** a C++17 compiler (Clang, GCC or MSVC), CMake ≥ 3.25, Ninja, and [vcpkg](https://github.com/microsoft/vcpkg) with the `VCPKG_ROOT` environment variable set. On macOS, also `brew install pkg-config`.

```bash
git clone https://github.com/Adiaharoni55/realtime-visual-slam.git
cd realtime-visual-slam

cmake --preset debug             # configure; vcpkg installs dependencies on first run
cmake --build --preset debug     # build
ctest --test-dir build/debug --output-on-failure   # run unit tests
./build/debug/rtvm               # run the app
```

Use the `release` preset for optimized builds. Debug builds run with AddressSanitizer and UndefinedBehaviorSanitizer.

## Data

Download the TUM RGB-D sequence [`freiburg1_xyz`](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download) (tgz) and unpack it into `data/`:

```bash
mkdir -p data && cd data
tar -xzf rgbd_dataset_freiburg1_xyz.tgz
```

The `data/` folder is not tracked by Git.

## Project structure

```
realtime-visual-slam/
├── include/rtvm/   # Public headers
├── src/            # Library implementation
├── apps/           # The rtvm command-line program
├── tests/          # Unit tests and small test fixtures
├── configs/        # Camera parameters (YAML)
├── scripts/        # Python helpers (evaluation, model export)
├── docs/           # Design notes
├── CMakeLists.txt
├── CMakePresets.json
└── vcpkg.json      # Dependencies
```

## References

- J. Sturm et al., *A Benchmark for the Evaluation of RGB-D SLAM Systems*, IROS 2012 (TUM RGB-D dataset).

## License

MIT