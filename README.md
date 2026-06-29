# stereo_depth_cpp

ROS2 C++ package that computes stereo depth from the Intel RealSense D455's IR1/IR2 cameras using OpenCV's `StereoBM` algorithm, and publishes the result as a `PointCloud2` topic. Built as an ARM-deployable alternative to the ELAS-based pipeline (`elas_ros2_cpp`, separate package/branch), since ELAS only supports x86 and was not viable for deployment on the Qualcomm QCS6490 dev kit.

This package was validated end-to-end: locally on x86, then cross-device (host PC publishing a recorded bag, QCS6490 dev kit subscribing and running the node over the network), confirming the full pipeline works before any cross-compilation effort.

---

## Why StereoBM instead of ELAS

| | ELAS | StereoBM |
|---|---|---|
| Platform support | x86 only | x86 + ARM |
| Compute cost | Higher (sparse support-point matching + triangulation + interpolation) | Lower (pure block matching, no learning/triangulation overhead) |
| Accuracy | Higher (validated ~57mm mean disparity vs RealSense SDK at ~4m, after baseline correction) | Lower, expected — not yet benchmarked against the same wall test at time of writing |
| Dependency footprint | Requires vendoring libelas C++ source (GPLv3) | Ships in standard OpenCV, zero extra dependencies |

StereoBM was chosen as the first ARM-compatible candidate to validate the deployment pipeline (build, QoS, cross-device networking, cross-compilation) before evaluating higher-accuracy alternatives (FastSGM, MobileStereoNet) that may need more engineering effort (e.g. NPU conversion pipelines).

---

## What this package does

- Subscribes to `/camera/camera/infra1/image_rect_raw` and `/camera/camera/infra2/image_rect_raw` (synchronized via `message_filters::ApproximateTime`)
  - **Note:** topic names may need remapping depending on the camera launch namespace in use — see [Topic remapping](#topic-remapping-note) below.
- Reads camera intrinsics from `/camera/camera/infra1/camera_info`
- Runs `cv::StereoBM` (`numDisparities=64`, `blockSize=15`) on each synchronized IR pair to compute a disparity map
- Converts disparity to depth using `Z = (fx * baseline) / disparity`, then back-projects to 3D `(X, Y, Z)` per pixel
- Publishes the resulting point cloud on `/custom/stereobm/points` (`sensor_msgs/msg/PointCloud2`, frame_id `camera_infra1_optical_frame`)
- Logs per-frame compute time (`compute_ms`) — the key metric for hardware feasibility comparisons across platforms
- Optionally saves every 10th frame's point cloud to disk as a flat binary file


---

## QoS configuration — required

This package will silently receive **zero messages** without correct QoS settings. This was the primary debugging effort during development; documented here so it isn't re-discovered the hard way.

### Subscriptions (camera topics)

RealSense camera drivers and `ros2 bag play` republish image/camera_info topics using the **Sensor Data QoS profile** — `BEST_EFFORT` reliability, small queue depth. A subscriber using the ROS2 default (`RELIABLE`) will connect to nothing: per the ROS2 design documentation, *"subscribing to a Best Effort publisher with a Reliable subscriber is not compatible and will result in no data being received on the subscriber."* No error is raised — the callback simply never fires.

**Fix applied:**
```cpp
auto qos = rclcpp::SensorDataQoS();

left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    this, "/camera/camera/infra1/image_rect_raw", qos.get_rmw_qos_profile());
right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    this, "/camera/camera/infra2/image_rect_raw", qos.get_rmw_qos_profile());
info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/camera/infra1/camera_info", qos,
    std::bind(&StereoBMNode::infoCallback, this, std::placeholders::_1));
```

### Publisher (this node's own output)

The output topic (`/custom/stereobm/points`) is kept on `RELIABLE` (the ROS2 default for publishers), so that generic subscribers — RViz, `save_frame.py`, etc. — connect without needing manual QoS adjustment on their end.

```cpp
rclcpp::QoS pub_qos(rclcpp::KeepLast(10));
pub_qos.reliable();
pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/custom/stereobm/points", pub_qos);
```

### How to check QoS on any live topic, rather than guessing

```bash
ros2 topic info /camera/camera/infra1/image_rect_raw --verbose
```

This prints the actual `RELIABILITY` and `DURABILITY` policy in use by a real running publisher — the reliable way to confirm compatibility before debugging blind.

---

## Topic remapping note

Depending on how the RealSense camera node was launched (namespace, `camera_name` parameter, etc.), the actual topic names on the network may differ from what this node subscribes to by default — e.g. `/camera/infra1/image_rect_raw` (single `camera`) vs `/camera/camera/infra1/image_rect_raw` (double `camera`, the RealSense ROS2 wrapper's default when `camera_name` and `camera_namespace` are both set to `camera`).

**Always verify before running:**
```bash
ros2 topic list | grep infra
```

**If names don't match, remap at runtime without rebuilding:**
```bash
ros2 run stereo_depth_cpp stereobm_node \
  --ros-args \
  -r /camera/camera/infra1/image_rect_raw:=/camera/infra1/image_rect_raw \
  -r /camera/camera/infra2/image_rect_raw:=/camera/infra2/image_rect_raw \
  -r /camera/camera/infra1/camera_info:=/camera/infra1/camera_info
```

---

## Build (native, x86)

```bash
cd ~/ros2_ws_stereo
colcon build --packages-select stereo_depth_cpp
source install/setup.bash
```

## Run — local validation

```bash
# Terminal 1
ros2 bag play <path_to_bag> (or) os2 launch realsense2_camera rs_launch.py enable_infra1:=true enable_infra2:=true pointcloud.enable:=true align_depth.enable:=true

# Terminal 2
ros2 run stereo_depth_cpp stereobm_node

# Terminal 3
rviz2
```

---

## Cross-device validation (host PC → QCS6490 dev kit, before cross-compiling)

This validates the networking/QoS/discovery path independently of the cross-compile toolchain, by running the **natively built x86 binary on the host** and a **natively built ARM binary on the dev kit** (built directly on-device, not cross-compiled), both communicating over the same network.

### 1. Set the same ROS_DOMAIN_ID on both machines
```bash
export ROS_DOMAIN_ID=42        # same value, both machines
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
```

### 2. Confirm reachability
```bash
ping <other_machine_ip>
```

### 3. Open firewall ports for DDS if active
```bash
sudo ufw allow 7400:7700/udp
```

### 4. Host PC: play the bag
```bash
source /opt/ros/humble/setup.bash
ros2 bag play <path_to_bag>
```

### 5. Dev kit: confirm topics and data are arriving over the network
```bash
ros2 topic list
ros2 topic hz /camera/infra1/image_rect_raw   # adjust name per Topic remapping note above
```

### 6. Dev kit: build natively and run
```bash
scp -r <path_to>/stereo_depth_cpp <devkit_user>@<devkit_ip>:~/ros2_ws/src/
# on dev kit:
cd ~/ros2_ws
colcon build --packages-select stereo_depth_cpp
source install/setup.bash
ros2 run stereo_depth_cpp stereobm_node
```

---

## Cross-compiling for QCS6490 (Qualcomm Robotics SDK toolchain)

Once native on-device builds are validated, cross-compilation avoids needing to build directly on the (typically slower) embedded target — useful for CI or repeated builds.

### Prerequisites

The Qualcomm Robotics ROS2 SDK toolchain must already be installed (the `.sh` installer extracted to a target directory, typically under `/opt/`). Source its environment setup script before building — this populates `SDKTARGETSYSROOT`, `OECORE_NATIVE_SYSROOT`, and `OE_CMAKE_TOOLCHAIN_FILE`:

```bash
source /opt/<qcom-sdk-install-path>/environment-setup-armv8-2a-qcom-linux
```

### Required environment exports before building

```bash
export AMENT_PREFIX_PATH=$SDKTARGETSYSROOT/usr:$AMENT_PREFIX_PATH
export CMAKE_PREFIX_PATH=$SDKTARGETSYSROOT/usr:$CMAKE_PREFIX_PATH
```

These ensure `colcon`/`ament_cmake` resolve ROS2 package dependencies (rclcpp, sensor_msgs, cv_bridge, message_filters, OpenCV) from the **target ARM sysroot**, not the host machine's x86 ROS2 install — otherwise the build will either fail to find ARM-compatible libraries or, worse, silently link against host x86 binaries.

### Clean build (always do this before a cross-compile attempt)

```bash
rm -rf build install log
```

A previous native (x86) build's `build/`/`install/` artifacts left in place will cause CMake to reuse stale, architecture-mismatched cache entries.

### Cross-compile build command

```bash
colcon build --merge-install --base-paths src --cmake-args \
  -DCMAKE_TOOLCHAIN_FILE="$OE_CMAKE_TOOLCHAIN_FILE" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF \
  -DCMAKE_FIND_ROOT_PATH="$SDKTARGETSYSROOT/usr" \
  -DCMAKE_PREFIX_PATH="$SDKTARGETSYSROOT/usr" \
  -DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=ONLY \
  -DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=NEVER \
  -DPython3_EXECUTABLE="$OECORE_NATIVE_SYSROOT/usr/bin/python3"
```

**What each flag does:**

| Flag | Purpose |
|---|---|
| `--merge-install` | Single shared `install/` output rather than per-package folders — simpler to `scp` to the target as one unit |
| `CMAKE_TOOLCHAIN_FILE` | Points CMake at the Yocto-generated toolchain file defining the ARM cross-compiler, sysroot, and target flags |
| `CMAKE_BUILD_TYPE=Release` | Optimized build — relevant for getting realistic `compute_ms` numbers, not debug-unoptimized timing |
| `BUILD_TESTING=OFF` | Skips building test targets not needed for deployment, reduces build time/dependency surface |
| `CMAKE_FIND_ROOT_PATH` | Tells CMake where to look for the target's libraries (the ARM sysroot), separate from the host's own `/usr` |
| `CMAKE_FIND_ROOT_PATH_MODE_PACKAGE=ONLY` | Forces `find_package()` calls to resolve **only** from the target sysroot — prevents accidentally linking host x86 ROS2 packages into an ARM binary |
| `CMAKE_FIND_ROOT_PATH_MODE_PROGRAM=NEVER` | Allows host-native tools (e.g. code generators that must run during the build itself) to still resolve from the host, not the target sysroot, since target-architecture binaries can't be executed on the host build machine |
| `Python3_EXECUTABLE` | Explicitly pins which Python interpreter (host-native) runs build-time scripts (e.g. ROS2 message generation), since auto-detection could otherwise pick an incompatible one |

### After a successful cross-compile

```bash
scp -r install/ <devkit_user>@<devkit_ip>:~/stereo_depth_cpp_install/
```

On the dev kit:
```bash
source ~/stereo_depth_cpp_install/setup.bash
ros2 run stereo_depth_cpp stereobm_node
```

---
