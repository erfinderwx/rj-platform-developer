# luchs_driver

> **Development preview 1.1.0-preview.1.** Adapted from the publicly distributed Luchs software v1.1.0 on 2026-10-11. Code/package versions remain 1.1.0 for source traceability. Tests supplied with the source are not evidence of successful execution or vehicle validation. Luchs B LCI is a draft interface; B configuration and firmware compatibility still require confirmation. See the [version development manual](../../docs/luchs-development-manual.md).

ROS 1 node `luchs_driver_node` for Luchs A and Luchs B.

```bash
roslaunch luchs_bringup luchs_a.launch build:=4ws dry_run:=true
roslaunch luchs_bringup luchs_b.launch dry_run:=true
```

Direct node startup requires loading the selected slash-key YAML; launch files provide model, driver parameters and overrides together.


Parameter files: `luchs_a_df.yaml`, `luchs_a_fa.yaml`, `luchs_a_da.yaml`, `luchs_a_4ws.yaml`, `luchs_b.yaml`.

Topics and services: see the [ROS 1 overview](../README.md). Parameter names use `/` rather than ROS 2 `.`. This local reference replaces the upstream dependency on a separately downloaded ROS 2 document.

## Parameters

| Name | Default | Unit / values | Meaning |
|---|---|---|---|
| `can_interface` | `can0` | | SocketCAN interface |
| `dry_run` | node: `false`; B YAML/launch: `true` | | built-in simulator instead of CAN; always pass/configure it explicitly when starting a node directly |
| `build` | `4ws` | `df` `fa` `da` `4ws` · `b` | Luchs A build or Luchs B; sets geometry, steering range, default protocol |
| `protocol/type` | `""` | `a` `lci` `legacy_b` | CAN protocol; empty = `a` for Luchs A, `lci` for Luchs B |
| `motion_model` | `""` | `diff` `ackermann` `dual_ackermann` `4ws` | model for `cmd_vel`; empty = default for the build (B: `dual_ackermann`) |
| `geometry/wheelbase` | A 0.594 · B 1.430 | m | |
| `geometry/track` | A 0.500 (4ws) / 0.596 · B 0.970 | m | |
| `geometry/wheel_radius` | A 0.103 · B 0.2794 | m | |
| `geometry/max_steer_angle` | A 1.5708 (4ws) / 0.60 (fa, da) · B 0.5236 | rad | per wheel / per axle |
| `limits/max_vx` / `max_vy` / `max_wz` | A 2.0 / 1.0 / 1.5 · B 2.0 / 0 / 1.0 | m/s, rad/s | command limits (host limits; B physical maximum is not validated by this preview) |
| `limits/max_accel_x` / `_y` / `_wz` | A 1.5 / 1.0 / 3.0 · B 1.0 / 0 / 1.0 | m/s², rad/s² | ramps |
| `limits/allow_pivot_turn` | A `true` · B `false` | | Ackermann builds turn on the spot for a pure yaw request |
| `control/rate` | A 20 · B 50 (LCI) / 100 (VCU) | Hz (1–100) | command cycle (LCI: heartbeat every cycle) |
| `control/cmd_timeout` | 0.5 | s | stop when commands stop |
| `control/feedback_timeout` | 0.5 | s | suppress commands when feedback stops |
| `control/park_when_idle` | `true` | | gear P when stopped and idle |
| `control/send_host_time` | `true` | | send 0x5A4 |
| `odom/frame_id` / `odom/child_frame_id` | `odom` / `base_footprint` | | |
| `odom/publish_tf` | `true` | | false when an EKF publishes the transform |
| `publish_rate` | 50 | Hz | state, odometry, joints |
| `cmd_vel_stamped` | `false` | | subscribe to `TwistStamped` |
| `joint_prefix` | `""` | | multi-robot |
| `battery_low_percent` | 15 | % | diagnostics warning |
| `battery/design_capacity` | A 15 · B 60 | Ah | `BatteryState.design_capacity` |
| `imu/frame_id` | `base_link` | | frame of `imu/data_raw` (LCI) |
| `protocol/*` | | | Luchs A firmware-dependent details, see [protocol reference](../docs/protocol_luchs_a.md) |
| `lci/follow_live_limits` | `true` | | apply the chassis' live limits (0x5B0) before sending, keeping path curvature |
| `lci/accel_limit` | 0 | m/s² | 0x5A1 AccelLimit; 0 = chassis default |
| `lci/speed_limit` | 0 | m/s | host speed cap in the heartbeat (also `~set_speed_limit`) |
| `lci/check_checksum` | `true` | | drop 0x5BD frames with a wrong checksum |
| `legacy_b/*` | | | Luchs B VCU interface V2.6: identifiers, steering sign, speed loop; see [protocol reference](../docs/protocol_luchs_b.md) |

## Services (Luchs B)

| Service | Type | |
|---|---|---|
| `~estop` | `std_srvs/SetBool` | e-stop request in the heartbeat (latched by the chassis until a maintenance reset) |
| `~park` | `std_srvs/SetBool` | controlled stop and hold (EPB) |
| `~set_payload_power` | `luchs_msgs/SetPayloadPower` | switched payload outputs 1 / 2, where fitted (LCI) |
| `~set_speed_limit` | `luchs_msgs/SetSpeedLimit` | host speed cap (LCI) |

On Luchs A these services answer `success: false` with the reason.

## Odometry

Integrated from the body twist the chassis reports in 0x5B5 (midpoint rule, every frame), at `base_link`. LCI reports the twist at the rear-axle centre; the driver converts it. Covariance: planar (x, y, yaw); z, roll, pitch unobserved. On the Luchs B VCU interface V2.6 the speed comes in 1 km/h steps and the yaw rate is computed from the commanded angle: a coarse estimate unsuitable as verified localization. LCI odometry and IMU are draft/software paths until hardware support is confirmed.


## B assumptions

The B geometry, battery capacity, VCU identifiers/steering sign and LCI hardware behavior are upstream defaults awaiting confirmation. The VCU current field is unresolved in the DBC and is not decoded by the SDK. Services encode host requests; a response from software is not proof of a physical action. See the [B interface status](../docs/protocol_luchs_b.md).
