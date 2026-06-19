# elas_ros2_cpp

ROS2 (Humble) C++ package that computes stereo depth from the Intel RealSense D455's IR1/IR2 cameras using the ELAS (Efficient Large-Scale Stereo Matching) algorithm, and publishes the result as a `PointCloud2` topic for visualization and comparison against the RealSense SDK's own depth output.

This package was built to validate an independent triangulation pipeline against Intel's built-in depth/point cloud generation, as part of the robotics data collection task at Nunnari Labs.

---

## What this package does

- Subscribes to `/camera/camera/infra1/image_rect_raw` and `/camera/camera/infra2/image_rect_raw` (synchronized via `message_filters::ApproximateTime`)
- Reads camera intrinsics from `/camera/camera/infra1/camera_info`
- Runs the ELAS stereo matching algorithm (libelas, `ROBOTICS` preset) on each synchronized IR pair to compute a disparity map
- Converts disparity to depth using `Z = (fx * baseline) / disparity`, then back-projects to 3D `(X, Y, Z)` per pixel using standard pinhole projection
- Publishes the resulting point cloud on `/custom/elas/points` (`sensor_msgs/msg/PointCloud2`, frame_id `camera_infra1_optical_frame`)
- Optionally saves every 10th frame's point cloud to disk as a flat binary file for offline analysis

---

## Repository structure

```
elas_ros2_cpp/
├── CMakeLists.txt              # Build configuration — compiles libelas sources alongside the node
├── package.xml                 # ROS2 package manifest and dependencies
├── include/
│   └── elas_ros2_cpp/
│       └── elas_node.hpp       # Node class declaration (subscriptions, publisher, ELAS instance, state)
├── src/
│   ├── main.cpp                 # Entry point — initializes rclcpp and spins the node
│   └── elas_node.cpp            # Node implementation — stereo callback, ELAS processing, point cloud publishing, disk save
└── third_party/
    └── libelas/                 # Vendored libelas C++ source (see below)
        ├── elas.cpp / elas.h
        ├── descriptor.cpp / descriptor.h
        ├── filter.cpp / filter.h
        ├── matrix.cpp / matrix.h
        ├── triangle.cpp / triangle.h
        └── timer.h
```

---

## third_party/libelas — where it came from and what's in it

The original libelas C++ implementation (Geiger, Roser, Urtasun — KIT) is not available as a standalone, actively maintained repo with a clean folder layout. It is most reliably obtained by cloning the Python binding repo **PieroV/PyElas** (https://github.com/PieroV/PyElas), which vendors the full original C++ source under its `src/` folder alongside the Python wrapper.

Steps used to populate this folder:

```bash
cd third_party/libelas
git clone https://github.com/PieroV/PyElas.git temp_pyelas

# Copy only the core C++ library files — exclude pyelas.cpp (the Python binding, not needed here)
cp temp_pyelas/src/elas.cpp .
cp temp_pyelas/src/elas.h .
cp temp_pyelas/src/descriptor.cpp .
cp temp_pyelas/src/descriptor.h .
cp temp_pyelas/src/filter.cpp .
cp temp_pyelas/src/filter.h .
cp temp_pyelas/src/matrix.cpp .
cp temp_pyelas/src/matrix.h .
cp temp_pyelas/src/triangle.cpp .
cp temp_pyelas/src/triangle.h .
cp temp_pyelas/src/timer.h .

rm -rf temp_pyelas
```

### File-by-file purpose

| File | Purpose |
|---|---|
| `elas.cpp` / `elas.h` | Core ELAS algorithm — the `Elas` class, `parameters` struct, `process()` entry point, support point matching, disparity computation |
| `descriptor.cpp` / `descriptor.h` | Sparse feature descriptor computation used to find reliable support points between the stereo pair |
| `filter.cpp` / `filter.h` | Image filtering utilities (e.g. Sobel-like gradient filters) used during descriptor/disparity computation |
| `matrix.cpp` / `matrix.h` | Lightweight custom matrix class used internally by ELAS for linear algebra operations |
| `triangle.cpp` / `triangle.h` | Delaunay triangulation of support points, used to interpolate dense disparity from sparse confident matches |
| `timer.h` | Optional profiling/timing utility (only active if `PROFILE` is defined; unused in this integration) |

**Note:** `pyelas.cpp` from the original repo (the Python/pybind11 binding layer) was deliberately **not** copied — it is not needed for the C++ build and would pull in Python dependencies unnecessarily.

### License

libelas is distributed under GPLv3 (see the original `LICENSE` file in PieroV/PyElas). This applies to the vendored source in `third_party/libelas/`.

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select elas_ros2_cpp
source install/setup.bash
```

---

## Run

Three terminals, in order:

```bash
# Terminal 1 — play back recorded sensor data
source /opt/ros/humble/setup.bash
ros2 bag play <path_to_bag>

# Terminal 2 — run the ELAS node
source ~/ros2_ws/install/setup.bash
ros2 run elas_ros2_cpp elas_node

# Terminal 3 — visualize
rviz2
```

In RViz, set **Fixed Frame** to `camera_infra1_optical_frame` and add a `PointCloud2` display subscribed to `/custom/elas/points`. To compare directly against the RealSense SDK's own cloud, also add a second `PointCloud2` display subscribed to `/camera/camera/depth/color/points` (only available if the bag was recorded with `pointcloud.enable:=true`).

---

## Key parameters

| Parameter | Location | Current value | Notes |
|---|---|---|---|
| `baseline_` | `elas_node.cpp` constructor | `0.095` m | Stereo baseline between IR1/IR2. Originally hardcoded as `0.05` from a prior reference implementation — this was found to be incorrect and caused ~640mm mean disparity error against RealSense ground truth. Corrected value (`0.095`) reduced mean disparity error to ~57mm at ~4m range. |
| ELAS preset | `elas_node.cpp` constructor | `Elas::ROBOTICS` | Alternative: `Elas::MIDDLEBURY`, tuned for benchmark datasets rather than robotics/outdoor scenes |
| `save_every_` | `elas_node.cpp` constructor | `10` | Saves point cloud to disk every Nth processed frame |
| `save_dir_` | `elas_node.cpp` constructor | `/mnt/hs2/SLAM-nav2/campus_bulding2/elas_live_cpp_output` | Output location for auto-saved per-frame point clouds |

If the baseline is ever re-derived (e.g. from a fresh calibration or a different camera unit), update `baseline_` in `elas_node.cpp` and rebuild.

---

## Output format (auto-saved point clouds)

Each saved file (`elas_points_<frame_number>.bin`) is a flat binary file of interleaved `float32` values: `X, Y, Z, X, Y, Z, ...` with no header.

To load in Python:

```python
import numpy as np

points = np.fromfile('elas_points_0010.bin', dtype=np.float32).reshape(-1, 3)
print(f"Loaded {len(points)} points")
```

---

## Validation summary

Compared against the RealSense SDK's own `/camera/camera/depth/color/points` output, on a flat wall section at ~4m depth, using nearest-neighbor point matching:

| Metric | Wrong baseline (0.05) | Corrected baseline (0.095) |
|---|---|---|
| Mean disparity | 640.90 mm | 56.96 mm |
| Median disparity | 637.99 mm | 53.90 mm |
| Max disparity | 1413.73 mm | 132.66 mm |
| Centroid offset | 739.73 mm | 70.83 mm |

The corrected baseline reduced error roughly 10x, bringing the result within normal stereo measurement tolerance (~1.8% of measured distance) at this range.

---

## Known limitations

- IMU/gyro/accel are not part of this node's scope — this package only consumes IR1/IR2 images and publishes a depth-derived point cloud.
- ELAS depth quality degrades at longer range (lower disparity resolution as distance increases with a fixed baseline) — best validated at distances under ~5m with the current D455 baseline.
- Stereo matching is inherently weaker on low-texture surfaces (e.g. flat painted walls); expect higher disparity/noise on such regions compared to textured scenes.
