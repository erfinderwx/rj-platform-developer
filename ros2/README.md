# Luchs ROS 2 — development preview

**Repository release: 1.1.0-preview.1 · upstream software/package version: 1.1.0.**
Adapted from the publicly distributed Luchs ROS 2 v1.1.0 package on 2026-10-11.
Chassis: RJ Tech Limited. Upstream software: TerraAgent Robotics (TAR).

This directory contains ROS 2 source packages for Luchs A and B. It is a development preview for code review, building and simulator-based integration. Supplied test code and CI configuration do not establish that a build or test has passed. Vehicle validation and the conditions for a final release are tracked in the [release documentation](../docs/luchs-development-manual.md).

| Item | Scope of this preview |
|---|---|
| ROS 2 targets | Humble / Ubuntu 22.04; Jazzy / Ubuntu 24.04; build and test results must be recorded for each |
| Luchs A | DF, FA, DA and 4WS configurations; protocol v1.1; firmware-dependent settings remain unit-specific |
| Luchs B LCI | Codec and simulator for LCI v1.0 draft r3; chassis firmware and hardware features are not established as implemented by this release |
| Luchs B VCU | V2.6 reference codec and host speed-loop implementation; identifiers, steering sign, bitrate, geometry and controller tuning need unit-specific confirmation |
| B launch default | `dry_run:=true`, using the built-in simulator rather than a CAN socket |
| Licence | [Apache-2.0](LICENSE), with upstream copyright notices retained |

## Source packages

| Directory | Content |
|---|---|
| [luchs_sdk](luchs_sdk/README.md) | ROS-independent C++17 SDK, protocol codecs, SocketCAN transport, steering kinematics, host watchdogs, simulators, CLI and Python examples |
| [luchs_driver](luchs_driver/README.md) | ROS 2 driver node/component, parameters and unit tests |
| `luchs_msgs` | State and command messages; motion-model, payload-power and speed-limit services |
| [luchs_description](luchs_description/README.md) | URDF/xacro and RViz setup; A-4WS visual meshes; B envelope model with unconfirmed geometry and IMU placement |
| `luchs_bringup` | Launch files and CAN/udev/systemd examples |
| `luchs_teleop` | Gamepad configuration |
| `luchs` | Metapackage |
| `dbc`, `docs` | Protocol references, DBC sources and integration notes; B draft/assumed fields are explicitly identified |

## Build the selected ROS directory

Clone the repository at the fixed preview release snapshot ref, then copy **only `ros2/`** into a ROS 2 workspace. Do not place the entire repository or both ROS generations into one workspace's `src/` directory.

```bash
git clone --branch luchs-v1.1.0-preview.1 https://github.com/erfinderwx/rj-platform-developer.git ~/rj-platform-developer
mkdir -p ~/luchs_ros2_ws/src
cp -a ~/rj-platform-developer/ros2 ~/luchs_ros2_ws/src/luchs_ros2
source /opt/ros/jazzy/setup.bash           # use humble on Ubuntu 22.04
cd ~/luchs_ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

This command sequence is supplied for the target Linux/ROS environment; this preview documentation does not claim it has passed on every listed target.

## Start with the simulator

```bash
ros2 launch luchs_bringup luchs_a.launch.py build:=4ws dry_run:=true
ros2 launch luchs_bringup luchs_b.launch.py dry_run:=true
ros2 launch luchs_bringup luchs_b.launch.py protocol:=legacy_b dry_run:=true
ros2 launch luchs_description display.launch.py config:=b
```

The B launch file and B YAML default to the simulator. A launch files retain upstream defaults, so use `dry_run:=true` explicitly for A development. Simulated feedback, IMU, limit changes, heartbeat stops and e-stop latching demonstrate the supplied software model; they do not demonstrate installed B hardware or validated stopping performance.

Before connecting B hardware, obtain the actual firmware/interface mapping, connector pinout, bitrate, termination, message IDs, steering sign, geometry and approved commissioning procedure. Setting `dry_run:=false` opens the hardware path and is an explicit commissioning decision. The intended customer LCI connector, IMU, fault definitions and other draft features must not be assumed available. See [CAN setup](docs/can_setup.md), [A protocol](docs/protocol_luchs_a.md) and [B interface status](docs/protocol_luchs_b.md).

## ROS interface

| Direction | Topic / service | Type / scope |
|---|---|---|
| Input | `cmd_vel` | `geometry_msgs/Twist`, or `TwistStamped` with `cmd_vel_stamped:=true` |
| Input | `~/drive_command`, `~/lamps` | `luchs_msgs/DriveCommand`, `LampCommand` |
| Output | `odom`, TF, `joint_states` | `nav_msgs/Odometry`, TF, `sensor_msgs/JointState` |
| Output | `~/state`, `~/safety`, `battery_state`, `/diagnostics` | Chassis/safety state, battery state and diagnostics |
| Output | `imu/data_raw` | LCI decoder/simulator path; B hardware availability, frame and mounting require confirmation |
| Service | `~/reset_odometry`, `~/set_motion_model` | `std_srvs/Trigger`, `luchs_msgs/SetMotionModel` |
| Service | `~/estop`, `~/park` | B host requests; physical outcome depends on validated chassis firmware |
| Service | `~/set_payload_power`, `~/set_speed_limit` | LCI draft requests; physical output support must be confirmed |

Parameter details are in [luchs_driver/README.md](luchs_driver/README.md). A and B use different wire protocols even though the SDK and ROS interfaces share source code. A shared package does not make the protocols interchangeable.

## B parameters and protocol limitations

- B defaults of 1.430 m wheelbase, 0.970 m track, 0.2794 m wheel radius and 30° road-wheel angle are assumptions awaiting chassis confirmation; the VCU command range is configured separately at 42°.
- The VCU sheet's `701` / `703` radix, steering direction and expected 500 kbit/s bitrate remain unconfirmed. Do not infer validity from the configured defaults.
- VCU speed feedback has 1 km/h steps and no actual steering/yaw feedback. Derived odometry uses the commanded steering angle and is a coarse estimate.
- The VCU DBC's current encoding/offset remains unconfirmed. The SDK does not decode this current field, and this preview adds no unsupported current decoder.
- The LCI 200 ms heartbeat threshold is a draft timeout requirement. It is not evidence that a real chassis completes a stop within 200 ms.
- B CAD, inertia and IMU placement in the envelope model are provisional. Do not use them as verified mounting or navigation-clearance data.

## Tests and reporting

The source includes 63 SDK unit tests and 7 ROS 2 driver parameter tests, plus protocol-level simulators and DBC checks. The repository's release documentation distinguishes tests present from tests actually executed and passed.

```bash
colcon test --packages-select luchs_sdk luchs_driver
colcon test-result --verbose
python3 src/luchs_ros2/dbc/check_lci_dbc.py   # requires cantools
```

ROS-free usage and examples: [luchs_sdk/README.md](luchs_sdk/README.md). Package-level changes: [CHANGELOG.md](CHANGELOG.md). Repository version, test evidence and final-release gates: [release documentation](../docs/luchs-development-manual.md).

For a public issue, provide the preview version, ROS target, build/configuration and a minimal sanitized reproduction. Do not publish original customer logs, private documents or credentials. Upstream contact and attribution are retained in the package metadata.
