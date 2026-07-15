# ROS 2 LiDAR-Inertial SLAM on a Raspberry Pi 4

A ROS 2 Humble stack for 2D mapping on a small differential-drive robot, built around
laser scan-matching odometry fused with an IMU. It runs entirely on the robot's
Raspberry Pi 4 — no external compute, no wheel encoders, no simulator.

The robot has no odometry source of its own: the Arduino only drives the motors and
reports nothing back. All motion estimation therefore comes from the LiDAR and the IMU,
which is the central constraint this stack is designed around.

Two configurations were built and compared: **LiDAR-only** (scan matching straight into
SLAM Toolbox) and **LiDAR + IMU** (scan matching fused with a filtered IMU through an
EKF). Both maps are in [`maps/`](maps/).

## Hardware

| Part | Detail |
|---|---|
| Compute | Raspberry Pi 4 Model B, 8 GB, Ubuntu 22.04, ROS 2 Humble (aarch64) |
| LiDAR | Slamtec RPLIDAR A2M7 — 12 Hz measured, 12 m max range |
| IMU | SparkFun ICM-20948, 9-DoF over I2C at 50 Hz (magnetometer unused) |
| Motor control | Arduino Uno over HC-05 Bluetooth serial, 4-wheel differential drive |

The Arduino side is a simple HC-05 + motor-driver sketch that just moves the bot around;
it takes no part in the ROS 2 stack and is not included here.

### Frames

```
base_link
 ├── laser      x=0     y=0  z=0.07
 └── imu_link   x=0.06  y=0  z=0.055
```

## Pipeline

```
RPLIDAR A2M7 ──/scan──┬─→ rf2o_laser_odometry ──/odom_rf2o──┐
                      │                                     ├─→ robot_localization EKF ──/odometry/filtered──┐
ICM-20948 ──/imu──→ imu_filter_madgwick ──/imu/data─────────┘         (30 Hz, 2D mode)                       │
                                                                                                             │
                      └──────────────────────/scan──────────────────────────────────────→ slam_toolbox ←─────┘
                                                                                              │
                                                                                          /map → map_saver_cli
```

`rf2o` publishes odometry only — its TF broadcast is patched off so it does not fight the
EKF over `odom`→`base_link`. The EKF owns that transform (see [Third-party](#third-party-dependencies)).

## Layout

```
src/my_robot_bringup/       launch + EKF and SLAM Toolbox configs
src/my_robot_description/   URDF and TF tree
tools/ros2_resmon.py        per-node CPU/RAM profiler used for the benchmarks
maps/                       saved occupancy grids from real runs
benchmarks/                 resmon CSV output behind the numbers below
docs/                       hardware and frame measurements
```

## Build

Requires ROS 2 Humble. The two upstream drivers are not vendored — fetch them pinned:

```bash
mkdir -p ~/lidar_slam_ws && cd ~/lidar_slam_ws
git clone https://github.com/Lochan25526/ros2-lidar-inertial-slam.git .
vcs import src < lidar_slam.repos
patch -p1 -d src/rf2o_laser_odometry < patches/rf2o_publish_tf_off.patch

rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

`icm20948_ros2` (the IMU driver) lives in the companion repo
[ros2-orbslam3-vio](https://github.com/Lochan25526/ros2-orbslam3-vio) and must be on your
`AMENT_PREFIX_PATH` for `sensors.launch.py` to start the IMU.

## Run

```bash
ros2 launch my_robot_bringup sensors.launch.py    # LiDAR + IMU + Madgwick filter
ros2 launch my_robot_bringup ekf.launch.py        # fuse rf2o odometry with IMU
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=$(ros2 pkg prefix my_robot_bringup)/share/my_robot_bringup/config/slam.yaml
```

Save a map once the area is covered:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab_map
```

Profile it:

```bash
python3 tools/ros2_resmon.py record --preset lidar_slam --duration 120
```

## Results

Mapping resolution 0.03 m/cell, `max_laser_range` 10 m. Measured with `ros2_resmon.py`
over 120 samples on the Pi 4 (raw CPU % is per-core; `norm` divides by 4 cores):

| Node | CPU mean | CPU peak | RSS mean |
|---|---|---|---|
| RPLIDAR driver | 68.0% | 72.6% | 21.5 MB |
| SLAM Toolbox | 1.9% | 9.5% | 55.1 MB |
| RF2O | 1.4% | 9.2% | 48.3 MB |
| EKF | 1.4% | 15.3% | 25.5 MB |
| Madgwick filter | 0.3% | 10.2% | 23.8 MB |
| robot_state_publisher | 0.2% | 6.1% | 23.1 MB |
| **Total workspace** | **72.6%** | **99.9%** | **181.8 MB** |

The headline finding: **the RPLIDAR driver alone accounts for ~94% of the stack's CPU**
(68.0 of 72.6%). SLAM itself is nearly free by comparison. On a 4-core Pi that is ~18% of
total capacity, so the platform is not compute-bound — but any optimisation effort belongs
in the driver, not the SLAM.

Raw per-sample data is in [`benchmarks/`](benchmarks/).

## Known issues and limitations

- **No wheel odometry.** The Arduino provides no encoder feedback, so `rf2o` scan matching
  is the only odometry source. In feature-poor or symmetric corridors it degrades, and the
  EKF has nothing independent to correct against.
- **The IMU contributes yaw only.** `ekf.yaml` takes just yaw and yaw-rate from `imu0`;
  `imu0_remove_gravitational_acceleration` is `false` and linear acceleration is unused.
  The fusion is therefore a yaw-stabilising aid, not full inertial odometry.
- **`use_mag: False`** — the ICM-20948 magnetometer is unused, so yaw has no absolute
  reference and is free to drift over long runs.
- `my_robot_description/urdf/robot.urdf.xacro` is named `.xacro` but contains no xacro
  macros, and `display.launch.py` reads it as a plain file. It works; the extension is
  misleading.
- `display.launch.py` starts `robot_state_publisher` only — despite the name, it does not
  start RViz. Use the config in `my_robot_description/rviz/` manually.

## Third-party dependencies

Neither is vendored; both are pinned in [`lidar_slam.repos`](lidar_slam.repos).

| Package | Upstream | Pin | Local change |
|---|---|---|---|
| `rplidar_ros` | [Slamtec/rplidar_ros](https://github.com/Slamtec/rplidar_ros) (BSD) | `24cc9b6` | none |
| `rf2o_laser_odometry` | [MAPIRlab/rf2o_laser_odometry](https://github.com/MAPIRlab/rf2o_laser_odometry) (GPL-3.0) | `b38c68e` | `publish_tf` default flipped to `false` |

The rf2o patch matters: upstream broadcasts `odom`→`base_link` by default, which collides
with `robot_localization` publishing the same transform (`publish_tf: true` in `ekf.yaml`).
Two publishers on one transform makes the TF tree jitter. The patch changes the default in
both the launch file and the `declare_parameter` call.

## License

MIT — see [LICENSE](LICENSE). Upstream dependencies keep their own licenses; note that
`rf2o_laser_odometry` is GPL-3.0 and is fetched, not redistributed here.

---

Built during a research internship at IIT Kharagpur, 2026.
