# GTSAM Legged Estimator ROS 2

ROS 2 implementation of four GTSAM legged-state-estimation variants from the
invariant-filter hierarchy. The node consumes IMU data, foot contact states, and
optionally joint states for Spot forward kinematics. It publishes odometry,
path, and foot contact markers for RViz, and writes CSV/TUM trajectory outputs
for later inspection.

For more details on the estimator variants, see the GTSAM blog post
[The Manifold Kalman Filter Hierarchy, Part 2: Legged State Estimation](https://gtsam.org/2026/03/17/legged-state-estimation-part2.html).

## Dependencies

This package was tested on Ubuntu 24.04 with ROS 2 Jazzy.

The main external dependencies are ROS 2 and GTSAM.

Required:

- Install GTSAM by following the official borglab repository:
  - https://github.com/borglab/gtsam

This package was tested with a source build of GTSAM from the `develop` branch.
The tested build reports `GTSAM_VERSION_STRING "4.3a1"` and
`GTSAM_VERSION_NUMERIC=40300`; this is a development build, not an official
GTSAM release.

## Build

Create a ROS 2 workspace, clone this package into its `src/` directory, and
build from the workspace root:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ChiyunNoh/GTSAM-Legged-Estimator-ROS2.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

If GTSAM is installed into a standard CMake search prefix such as `/usr` or
`/usr/local`, no extra path setting is usually needed. If GTSAM is installed into
a custom prefix, expose it before running `colcon build`:

```bash
export CMAKE_PREFIX_PATH=/path/to/gtsam/install:$CMAKE_PREFIX_PATH
```

Alternatively, point CMake directly at the GTSAM package config directory:

```bash
export GTSAM_DIR=/path/to/gtsam/install/lib/cmake/GTSAM
```

## Dataset

The experiments use the dataset from
[GaRLILEO: Gravity-aligned Radar-Leg-Inertial Enhanced Odometry](https://garlileo.github.io/GaRLILEO/)
(IJRR 2026, Noh et al.).
The GaRLILEO dataset contains ROS 2 bag sequences collected with a Boston
Dynamics Spot robot equipped with IMU, leg kinematics, and radar sensors.

## Run

In one terminal, replay a GaRLILEO ROS 2 bag:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 bag play /path/to/garlileo_sequence
```

In another terminal, launch the estimator with RViz:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch gtsam_legged_replay_example GTSAM_legged_estimator.launch.xml
```

## Input Topics

Defaults are defined in `config/GTSAM_legged_estimator.yaml`.

```text
/imu                sensor_msgs/msg/Imu
/spot/status/feet  spot_msgs/msg/FootStateArray
/joint_states      sensor_msgs/msg/JointState
```

When `topics.read_joint_states: true`, the node uses `/joint_states` to compute
Spot foot positions with forward kinematics. If this is disabled or the joint
state is too old, the node falls back to the body point contained in the foot
state message when configured to do so.

## Output Topics

Exactly one estimator variant is allowed per live run. Output topics are fixed
so the same RViz configuration works even when the selected variant changes.

```text
/legged_estimator/odom           nav_msgs/msg/Odometry
/legged_estimator/path           nav_msgs/msg/Path
/legged_estimator/foot_contacts  visualization_msgs/msg/MarkerArray
```

If `estimator.variants` contains zero or multiple variants, the node exits at
startup with an error.

```yaml
estimator:
  variants: [invariant_ekf]
```

Supported variant names:

```text
invariant_ekf
invariant_graph
fixed_lag_single_bias
fixed_lag_combined_bias
```

## Output Files

The node writes outputs under `estimator.output_dir`, which defaults to
`./outputs/live`.

```text
<variant>_trajectory.csv
<variant>_trajectory_imu.tum
<variant>_metrics.csv
```

The `outputs/` directory is ignored by Git because these files are generated
runtime artifacts.

## License

This project is licensed under the BSD-3-Clause License, following GTSAM's
license terms. See the repository root `LICENSE` for details.
