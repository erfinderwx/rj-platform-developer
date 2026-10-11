# Luchs ROS 1 — development preview

**Repository release: 1.1.0-preview.1 · upstream software/package version: 1.1.0.**
Adapted from the publicly distributed Luchs ROS 1 v1.1.0 package on 2026-10-11.
Chassis: RJ Tech Limited. Upstream software: TerraAgent Robotics (TAR).

This directory is the ROS 1 Noetic counterpart of the ROS 2 source, using the same C++ SDK and protocol references. This is a development preview. Test files supplied with the package do not prove they have passed; B firmware, geometry and protocol assumptions still need confirmation. See the [release documentation](../docs/luchs-development-manual.md) for target validation and final-release conditions.

| Item | Scope of this preview |
|---|---|
| Target | ROS 1 Noetic / Ubuntu 20.04, C++17; Melodic is not tested |
| Luchs A | DF, FA, DA and 4WS reference configurations, protocol v1.1 |
| Luchs B | LCI v1.0 draft r3 codec/simulator and VCU V2.6 reference implementation; not a claim of final firmware or vehicle validation |
| B launch default | `dry_run:=true`, using the simulator rather than a CAN socket |
| Licence | [Apache-2.0](LICENSE), upstream notices retained |

## Source packages

`luchs_sdk`, `luchs_msgs`, `luchs_driver`, `luchs_description`, `luchs_bringup`, `luchs_teleop` and `luchs` contain the SDK, messages/services, ROS 1 driver, model, launch/setup examples, gamepad configuration and metapackage. `dbc/` and `docs/` contain the protocol references. Driver behavior and slash-separated parameters are documented locally in [luchs_driver/README.md](luchs_driver/README.md).

## Build the selected ROS directory

Clone at the fixed preview release snapshot ref, then copy **only `ros1/`** into a ROS 1 workspace. Do not put the whole repository or the ROS 2 directory into that workspace. The SDK is a plain CMake package, so use `catkin build` or `catkin_make_isolated`; plain `catkin_make` does not support this mixed workspace.

```bash
git clone --branch luchs-v1.1.0-preview.1 https://github.com/erfinderwx/rj-platform-developer.git ~/rj-platform-developer
mkdir -p ~/luchs_ros1_ws/src
cp -a ~/rj-platform-developer/ros1 ~/luchs_ros1_ws/src/luchs_ros1
source /opt/ros/noetic/setup.bash
cd ~/luchs_ros1_ws
rosdep install --from-paths src --ignore-src -y
catkin build                            # requires python3-catkin-tools
source devel/setup.bash
```

This command sequence targets the listed Linux/ROS environment. The release documentation records actual validation; this README does not declare Noetic execution successful solely because the files are present.

## Start with the simulator

```bash
roslaunch luchs_bringup luchs_a.launch build:=4ws dry_run:=true
roslaunch luchs_bringup luchs_b.launch dry_run:=true
roslaunch luchs_bringup luchs_b.launch protocol:=legacy_b dry_run:=true
roslaunch luchs_description display.launch config:=b
```

The B launch file and B YAML default to simulation. A launch files retain upstream defaults; pass `dry_run:=true` explicitly for A development. Simulated LCI heartbeat, IMU, e-stop and live-limit behavior is not evidence that those B hardware functions are installed or validated.

Before B hardware commissioning, confirm the actual firmware/interface, connector pinout, bitrate, CAN termination, identifiers, steering sign, geometry and approved procedure. `dry_run:=false` explicitly selects the hardware path. Do not assume the dedicated LCI bus or its planned hardware is fitted. References: [CAN setup](docs/can_setup.md), [A protocol](docs/protocol_luchs_a.md), [B draft and VCU limitations](docs/protocol_luchs_b.md).

## ROS interface

| Direction | Topic / service | Type / scope |
|---|---|---|
| Input | `cmd_vel`, `~drive_command`, `~lamps` | Twist/TwistStamped, `DriveCommand`, `LampCommand` |
| Output | `odom`, TF, `joint_states`, `~state`, `~safety` | Odometry, TF, joints and chassis/safety state |
| Output | `battery_state`, `/diagnostics`, `imu/data_raw` | IMU is available through the LCI decoder/simulator; physical B implementation is unconfirmed |
| Service | `~reset_odometry`, `~set_motion_model` | `std_srvs/Trigger`, `SetMotionModel` |
| Service | `~estop`, `~park` | B host requests; real effect depends on confirmed firmware |
| Service | `~set_payload_power`, `~set_speed_limit` | LCI draft requests, requiring confirmed hardware/firmware |

ROS 1 uses `/` in parameter names where ROS 2 uses `.`. Shared software does not make A and B wire protocols interchangeable. B geometry, VCU message radix/steering sign/current offset and LCI hardware behavior retain their unconfirmed status; the SDK does not decode the unresolved VCU current field. VCU odometry is coarse because feedback has 1 km/h resolution and no measured steering angle. See the local [driver parameters](luchs_driver/README.md).

## Tests and reporting

Two ROS 1 dry-run rostests and 63 shared SDK tests are supplied. They must be executed in the correct environment and their results recorded; their presence is not a pass result.

```bash
catkin build && source devel/setup.bash
rostest luchs_driver dry_run_a.test
rostest luchs_driver dry_run_b.test
```

SDK usage: [luchs_sdk/README.md](luchs_sdk/README.md). Package changes: [CHANGELOG.md](CHANGELOG.md). Version, validation evidence and final-release conditions: [release documentation](../docs/luchs-development-manual.md).

Public issues should contain the preview version, target, configuration and a minimal sanitized reproduction. Do not upload private material, customer data, credentials or original logs. Upstream attribution and maintainer metadata are retained.
