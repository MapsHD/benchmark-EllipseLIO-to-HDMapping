# benchmark-EllipseLIO-to-HDMapping

Runs the [EllipseLIO](https://github.com/v4rl-ucy/ellipselio) LiDAR-Inertial
odometry algorithm on a ROS 2 bag file and converts the output to an
[HDMapping](https://github.com/MapsHD/HDMapping) session.

EllipseLIO is an *Adaptive LiDAR Inertial Odometry approach with an Ellipsoid
Representation*, by V4RL, ETH/UCY.

## Prerequisites

- Docker
- A ROS 2 bag containing a `sensor_msgs/msg/PointCloud2` topic and a
  `sensor_msgs/msg/Imu` topic matching the topic names declared in the chosen
  EllipseLIO config file (ROS 1 bags are automatically converted to ROS 2 format).

## Step 1 — Clone with submodules

```bash
git clone https://github.com/MapsHD/benchmark-EllipseLIO-to-HDMapping.git --recursive
cd benchmark-EllipseLIO-to-HDMapping
```

## Step 2 — Build the Docker image

```bash
docker build -t ellipselio_humble .
```

This installs:
- Ubuntu 22.04 + ROS 2 Humble
- Eigen3, PCL, OpenCV, Boost, OpenMP
- EllipseLIO (compiled from submodule)
- ROS 2 workspace with `ellipselio` and `ellipselio-to-hdmapping`

The build takes several minutes on first run.

## Step 3 — Run the pipeline

```bash
chmod +x docker_session_run-ros2-ellipselio.sh
./docker_session_run-ros2-ellipselio.sh /path/to/input.bag /path/to/output/dir
```

Or with no arguments to use a GUI file selector (requires `zenity`):

```bash
./docker_session_run-ros2-ellipselio.sh
```

By default the script uses the `qt64_spires.yaml` EllipseLIO config. Pick a
different one with the `CONFIG_FILE` environment variable, e.g.:

```bash
CONFIG_FILE=os64_ncd.yaml ./docker_session_run-ros2-ellipselio.sh /path/to/input.bag /path/to/output/dir
```

Available configs:

Upstream EllipseLIO ships these in `src/ellipselio/config/`:

| Config | LiDAR | Typical dataset |
|--------|-------|-----------------|
| `qt64_spires.yaml`  | Hesai Pandar QT64    | Oxford Spires    |
| `os64_ncd.yaml`     | Ouster OS-64         | Newer College    |
| `os128_ncd.yaml`    | Ouster OS-128        | Newer College    |
| `os64_geode.yaml`   | Ouster OS-64         | GEODE            |
| `vlp16_bot.yaml`    | Velodyne VLP-16      | BotanicGarden    |
| `vlp16_geode.yaml`  | Velodyne VLP-16      | GEODE            |
| `vlp16_graco.yaml`  | Velodyne VLP-16      | GraCo            |

This wrapper ships additional configs (overrides the upstream share dir when
the file name matches):

| Config | LiDAR | Typical dataset | Notes |
|--------|-------|-----------------|-------|
| `livox_reg.yaml`    | Livox Mid-360        | Bunker-DVI registration set | Stored in [config/livox_reg.yaml](config/livox_reg.yaml). Requires the `livox_format_bridge` node — launched automatically. |

### Bunker-DVI (Livox Mid-360) — branch `Bunker-DVI-Dataset-reg-1`

On this branch `livox_reg.yaml` is the **default**, so the simplest invocation
is just:

```bash
./docker_session_run-ros2-ellipselio.sh /path/to/reg-1.bag-pc.bag /path/to/output/dir
```

Two wrapper-side mechanisms make this work *without modifying the upstream
algorithm submodule*:

1. **Wrapper config.** [config/livox_reg.yaml](config/livox_reg.yaml) is mounted
  read-only at `/ros2_ws/wrapper_config/livox_reg.yaml`, and the run script
  passes `config_path:=/ros2_ws/wrapper_config` so the algorithm loads our yaml
  instead of the upstream package share. The yaml pins the per-bin octree
   resolution sensibly for the Mid-360 non-repetitive scanner
   (`scan_lines: 128` — *not* the physical line count; this parameter drives
   the voxel size in `LidarProcess`), uses the BMI088 noise/bias figures, and
  subscribes to `/livox/pointcloud_native` and `/livox/imu`.

2. **`livox_format_bridge`** (in [src/ellipselio-to-hdmapping/src/livox_format_bridge.cpp](src/ellipselio-to-hdmapping/src/livox_format_bridge.cpp))
   republishes `/livox/pointcloud` (ROS 1 `livox_ros_driver` layout, with
   `float time` = per-point offset from `header.stamp`) as
   `/livox/pointcloud_native` in the upstream-algorithm-native layout
  (`double timestamp` = absolute time in nanoseconds, matching Humble
  `rclcpp::Time(timestamp)`). The node is launched automatically whenever the
  selected config is `livox_reg.yaml`, and logs a heartbeat every 50 forwarded
  clouds.

The upstream `src/ellipselio` submodule is **never patched** by this wrapper.

**What happens:**

The script opens a Docker container with a tmux session containing five panes
and a control window:

| Pane | Role |
|------|------|
| 0 | `ros2 launch ellipselio ellipselio_standalone.launch.py` — subscribes to the LiDAR + IMU topics from the chosen config, publishes `/ellipselio_odom` + `/cloud_scan` |
| 1 | RViz2 — live visualization of the EllipseLIO map and trajectory |
| 2 | `ros2 bag record` — captures the two published topics |
| 3 | `ros2 bag play` — plays your input bag with simulated clock |
| 4 | diagnostics — shows active topics and publishing rates |
| control | auto-shutdown — waits for playback to finish, then stops all nodes |

After playback completes, the control window automatically stops the recorder,
kills all nodes, and exits tmux. A second Docker run then converts the recorded
bag into the HDMapping session format.

## Step 4 — Open in HDMapping

Output files appear in `<output_dir>/output_hdmapping-EllipseLIO/`:

```
lio_initial_poses.reg
poses.reg
scan_lio_0.laz
scan_lio_1.laz
...
session.json
trajectory_lio_0.csv
trajectory_lio_1.csv
...
```

Open `session.json` with the
[multi_view_tls_registration_step_2](https://github.com/MapsHD/HDMapping)
application.

## Notes on EllipseLIO

EllipseLIO requires both LiDAR point clouds and IMU data. The input topic names
are configured **inside the YAML config file** (not as command-line parameters
like D-LIO). The relevant fields are:

```yaml
lidar:
  type: 1|2|3|4   # LIVOX, VELODYNE, OUSTER, HESAI
  rate: 10
  topic: "/your/lidar/topic"
  t_imu_lidar: [x, y, z]
  r_imu_lidar: [qx, qy, qz, qw]
imu:
  rate: 200
  topic: "/your/imu/topic"
```

If the topic names in your bag differ from the ones in the chosen config,
either pick another config or mount a customised config into the container.

The recorded topics (used by the converter) are also tunable via env vars:

| Variable | Meaning | Default |
|----------|---------|---------|
| `ODOM_TOPIC`  | EllipseLIO odometry output | `/ellipselio_odom` |
| `CLOUD_TOPIC` | EllipseLIO per-scan cloud (world frame) | `/cloud_scan` |

Note: EllipseLIO publishes `/cloud_scan` already expressed in the
`odom_ellipselio` (world) frame, so the converter does **not** re-apply the
odometry to the points — it only uses `/ellipselio_odom` to build the
per-chunk trajectory files.

## Contact

januszbedkowski@gmail.com
