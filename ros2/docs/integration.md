# Integration guide

> **Development preview 1.1.0-preview.1.** Adapted from the publicly distributed Luchs software v1.1.0 on 2026-10-11. Code/package versions remain 1.1.0 for source traceability. Tests supplied with the source are not evidence of successful execution or vehicle validation. Luchs B LCI is a draft interface; B configuration and firmware compatibility still require confirmation. See the [version development manual](../../docs/luchs-development-manual.md).

## Mounting a sensor (Luchs A reference)

Two 2040 aluminium rails run along the deck, 200 mm apart (centre to centre); use 40-series slot nuts. The URDF has frames `rail_left_link` and `rail_right_link` on top of the rails at mid-length. Add a sensor in your own xacro:

```xml
<xacro:include filename="$(find luchs_description)/urdf/luchs_a_macro.xacro"/>
<xacro:luchs_a config="4ws" prefix=""/>
<joint name="lidar_joint" type="fixed">
  <parent link="rail_left_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.25 -0.10 0.08" rpy="0 0 0"/>   <!-- measured on your robot -->
</joint>
<link name="lidar_link"/>
```

Payload power: 48 V / 10 A and 12 V / 8 A on the power expansion port.

## Sensor fusion (robot_localization)

Set `odom.publish_tf: false` in the driver and let the EKF publish `odom → base_footprint`:

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true
    odom_frame: odom
    base_link_frame: base_footprint
    world_frame: odom
    odom0: /odom
    odom0_config: [false, false, false,  false, false, false,
                   true,  true,  false,  false, false, true,
                   false, false, false]   # vx, vy, yaw rate
    imu0: /imu/data
    imu0_config:  [false, false, false,  false, false, false,
                   false, false, false,  false, false, true,
                   true,  false, false]   # yaw rate, forward acceleration
```

For the skid-steer and Ackermann builds set `vy` to false.

## GNSS / INS (Fixposition Vision-RTK 2)

The Fixposition ROS 2 driver can feed the chassis odometry to the sensor as wheel-speed input with its built-in converter — no extra node:

```yaml
fixposition_driver:
  ros__parameters:
    converter:
      enabled: true
      topic_type: "Odometry"
      input_topic: "/odom"
      scale_factor: 1000.0     # m/s -> mm/s
      use_x: true
      use_y: false             # true on the A-4WS if you drive sideways
      use_z: false
```

Mount the sensor and antennas on the rails, enter the lever arms in the sensor web interface, and use the sensor's `odom → base` output instead of the driver's TF (`odom.publish_tf: false`).

## Navigation (Nav2)

- Robot footprint: A-4WS 0.80 × 0.58 m; A-DF / A-FA / A-DA 0.80 × 0.665 m; **B provisional envelope 1.99 × 1.15 m** (not a verified collision footprint).
- Controller: Regulated Pure Pursuit or MPPI. For the Ackermann builds use the Ackermann motion model and a minimum turning radius of 0.89 m (A-FA) / 0.445 m (A-DA); the A-4WS and A-DF can rotate in place. MPPI with the `Omni` model uses the A-4WS's sideways motion.
- **Luchs B** steers like a car on both axles: MPPI `Ackermann` model or Regulated Pure Pursuit, no in-place rotation. Minimum turning radius at `base_link` with 30° road-wheel angle: 1.24 m (dual Ackermann), 2.48 m at the rear axle (front Ackermann); confirm on the vehicle. Feed `~/state.max_vx_forward` into the planner if you want it to plan with the chassis' live derating, or let the driver clip (path curvature is kept).
- Planned Luchs B chassis IMU (hardware and mounting unconfirmed): `imu/data_raw` (LCI 0x5BE / 0x5B1) is represented in the decoder/simulator and may be integrated with robot_localization only after confirmed firmware, axes, mounting and validation as `imu0` (angular velocity only; orientation is not provided).
- Velocity smoother in front of the driver; the driver's own acceleration limits are a safety net, not a substitute.
- Keep `cmd_vel` coming at ≥ 5 Hz while moving (driver watchdog 0.5 s).

## Several robots

Use `namespace:=r1 prefix:=r1_` on the launch file: topics go under `/r1`, TF frames become `r1_base_footprint` etc.
