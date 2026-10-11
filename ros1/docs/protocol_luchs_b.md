# Luchs B CAN interfaces — reference for the driver

> **Development preview 1.1.0-preview.1.** Adapted from the publicly distributed Luchs software v1.1.0 on 2026-10-11. Code/package versions remain 1.1.0 for source traceability. Tests supplied with the source are not evidence of successful execution or vehicle validation. Luchs B LCI is a draft interface; B configuration and firmware compatibility still require confirmation. See the [version development manual](../../docs/luchs-development-manual.md).

> ROS 1 (`luchs_ros1`): parameter names use `/` where the ROS 2 driver uses `.` (`protocol/type`, `lci/follow_live_limits`); launch files are `luchs_a.launch` / `luchs_b.launch`.

The source implements two Luchs B interface paths. This preview does not establish the installed interface or firmware on any vehicle; choose with `protocol/type` (or `protocol:=` on `luchs_b.launch`).

| | **LCI v1.0** (`lci`, default) | **VCU upper interface V2.6** (`legacy_b`) |
|---|---|---|
| Status | Draft r3; firmware/hardware implementation unconfirmed | V2.6 reference; unit-specific parameters and firmware mapping unconfirmed |
| Bus | Dedicated customer CAN, 1 Mbit/s | VCU bus, bit rate to be confirmed (500 kbit/s expected) |
| Byte order | Big-endian | Little-endian |
| Command level | Speed + yaw rate, or speed + axle angles; the chassis closes the loops | Accelerator / brake pedal %; the host closes the speed loop |
| Feedback | Body motion (0.001 m/s), axle angles, wheel speeds, live limits, IMU, EPB / EPS / brake, temperatures, version | Speed in whole km/h, gear, battery, fault level |
| Host liveness | Heartbeat 0x5A5 every 20 ms with counter and checksum; draft 200 ms timeout threshold; stop/park behavior and stopping time require validation | Heartbeat byte in 0x701 |
| Preview scope | Draft codec and simulator development; autonomy requires confirmed firmware and validation | Reference codec/simulator; hardware commissioning and tuning still required |

The source has a shared LCI code path for the Luchs family. This is not evidence that LCI firmware is available on Luchs A or B; select an interface only after confirming the vehicle firmware.

---

## LCI v1.0 (draft r3)

CAN 2.0, **1 Mbit/s**, 11-bit identifiers, DLC 8, **big-endian**, signed values in two's complement. Physical value = raw × factor + offset. `dbc/luchs_lci_v1.0_draft_r3.dbc` and `Luchs_B_CAN_Protocol_LCI_v1.0_draft.xlsx` describe the same frames; `dbc/check_lci_dbc.py` checks the DBC against the SDK's test vectors.

**Reference point:** rear-axle centre on the ground, X forward, Y left, Z up. The driver converts to `base_link` (centre of the wheelbase): `vy(base_link) = vy(rear axle) + wz · L/2`. In dual Ackermann the rear-axle centre slides sideways (`vy = −wz · L/2`); `base_link` does not.

### Host → chassis

| ID | Name | Cycle | Bytes |
|---|---|---|---|
| 0x5A0 | MotionCmd | 20 ms | 0–1 TargetVx · 2–3 TargetVy (4WS only, else 0) · 4–5 TargetWz (int16, 0.001 m/s, rad/s) · 6 KinematicModel (2 front Ackermann, 4 dual Ackermann) · 7 ControlMode 0x00 |
| 0x5A1 | SteerCmd | 20 ms | 0 Gear (0 disabled, 1 P, 2 R, 3 N, 4 D) · 1–2 front angle · 3–4 rear angle (int16, 0.001 rad) · 5 SteerSource (0 from yaw rate, 1 to these angles, also at standstill) · 6–7 AccelLimit (uint16, 0.001 m/s², 0 = default) |
| 0x5A3 | LampCmd | 100 ms | 0 turn (0 off, 1 left, 2 right, 3 hazard) · 1 reverse · 2 fault · 3 headlight (0 off, 1 low, 2 high) · 4 horn · 5 beacon |
| 0x5A4 | HostTime | 1 s | 0–7 Unix time in ms, uint64 |
| 0x5A5 | HostHeartbeat | 20 ms | 0 AliveCounter (+1 per frame) · 1 bit 0 EStopRequest, bit 1 ParkRequest, bits 3/4 payload power 1/2 · 2–3 SpeedLimit (uint16, 0.01 m/s, 0 = default) · 7 checksum = (sum of bytes 0–6) & 0xFF XOR 0xFF |

### Chassis → host

| ID | Name | Cycle | Content |
|---|---|---|---|
| 0x5B0 | LimitsFb | 20 ms | max forward / reverse speed (0.01 m/s), max road-wheel angle (0.001 rad), LimitReason bits, clipped flags |
| 0x5B1 | ImuAccelFb | 10 ms | accelerations X/Y/Z (0.002 m/s², gravity included), counter |
| 0x5B2 | AttitudeFb | 20 ms | roll, pitch (0.0001 rad) |
| 0x5B5 | MotionFb | 20 ms | as 0x5A0; byte 7: 0x11 remote control, 0x22 automatic |
| 0x5B6 | SteerFb | 20 ms | gear, front and rear axle angles |
| 0x5B8 | ProtectionFb | 50 ms | ultrasonic trigger, bumper, e-stop button, fault code, four ultrasonic distances (cm, 255 = none) |
| 0x5B9 | WheelSpeedFb | 20 ms | LF, RF, LR, RR (int16, 0.01 rad/s) |
| 0x5BA | MotorFault | 100 ms | two 32-bit fault words |
| 0x5BB | BatteryFb | 100 ms | SOC (0.01 %), temperature (0.1 °C), charging progress (%), voltage (0.1 V) |
| 0x5BD | ChassisState | 20 ms | operating mode (0 maintenance, 1 RC, 2 automatic), EStopSource bits, fault level (0–3), RC online / enabled / link, 4WD, park brake, clipped, active speed limit, alive counter, checksum |
| 0x5BE | ImuRateFb | 10 ms | roll / pitch / yaw rate (0.001 rad/s), counter, IMU status |
| 0x5BF | Version | 1 s | LCI major.minor, ModelId (0x1B = Luchs B), firmware major.minor.patch |
| 0x5C0 | B_BrakeSteerFb | 50 ms | brake pressure (0.01 MPa), EPB state, front / rear EPS state, brake state |
| 0x5C1 | B_MotorTempFb | 500 ms | hub-motor and controller temperatures (1 °C, offset −40) |

### What the driver does with it

- Sends 0x5A0, 0x5A1 and the heartbeat every cycle (50 Hz), lamps at 10 Hz, host time at 1 Hz. The draft specifies loss/freeze detection after 200 ms and a controlled stop/park request. Actual fault detection, deceleration, stopping time and EPB application must be verified on the applicable firmware; this is not a guarantee of a completed stop within 200 ms.
- Applies the live limits from 0x5B0 before sending (`lci/follow_live_limits`). A speed reduced by a limit keeps its path curvature, so a planned path is still followed, just slower. Limits older than 200 ms are ignored.
- `DriveCommand` with explicit axle angles sets SteerSource = 1 (angle mode). `cmd_vel` uses SteerSource = 0; the driver still sends the matching angles so every frame of a cycle describes the same motion.
- Ignores 0x5BD frames with a wrong checksum and counts them, together with counter jumps, in diagnostics.
- Compares 0x5BF ModelId with the configured build and reports a mismatch as a diagnostics error.
- Services `~/estop`, `~/park`, `~/set_payload_power`, `~/set_speed_limit` set the heartbeat fields. Latching and maintenance-reset behavior are modeled by the simulator and specified in the draft; obtain a validated chassis recovery procedure before hardware use.
- Publishes `imu/data_raw` (sensor_msgs/Imu, no orientation) from 0x5BE / 0x5B1.

---

## VCU upper interface V2.6 (`legacy_b`)

Little-endian. `dbc/luchs_b_vcu_v2.6.dbc` (generated and checked by `dbc/gen_dbc_luchs_b_vcu.py`).

| ID | Direction | Content |
|---|---|---|
| 0x701 UpperCmd | host → VCU, 10 ms | 0 accelerator % · 1 brake % · 2 road-wheel angle (int8, deg, ±42) · 3 speed cap (km/h, ≤ 30) · bits 32–41 steering rate (°/s, 50–520) · bit 48 upper-computer control · 49 headlight · 50 e-stop · 51 park · bits 52–55 gear (0 N, 1 D, 2 R) · 7 heartbeat |
| 0x703 VCU2Upper | VCU → host, 10 ms | 0 SOC % · 1 battery temperature (−40) · 2 voltage (V) · bits 24–25 fault level (0 none, 1 motor controller, 2 BMS, 3 several) · bits 32–41 current · bits 42–43 gear · bit 44 mode (1 upper computer) · 6 speed (km/h, unsigned) · 7 heartbeat |

What the driver does: a PI speed loop with feed-forward on the accelerator, brake on overspeed, a hold brake at rest, and the VCU speed cap set just above the target as a second limit. Direction changes only at standstill and only after the VCU reports the new gear. Odometry uses the reported speed (1 km/h steps) and the **commanded** steering angle, so it is coarse; use it for teleoperation and recording, not for localisation.

### Pending confirmation before hardware commissioning

| Parameter | Default | How to check |
|---|---|---|
| `legacy_b/cmd_id` / `fb_id` | 0x701 / 0x703 (the sheet writes "701") | `candump can0`: the VCU frame every 10 ms is 0x703 or 0x2BF |
| `legacy_b/steer_sign` | −1 (wire positive = steer right) | request the manufacturer-confirmed sign and approved commissioning procedure; do not treat this default as measured |
| bit rate | 500 kbit/s expected | `sudo ./setup_can.sh can0 500000`, then `candump` shows clean frames |
| `legacy_b/throttle_*`, `brake_kp` | conservative | requires controlled hardware commissioning and documented tuning; no validated gains are supplied |

---

## Geometry (to be confirmed by the chassis team)

| Item | Driver default | Open point |
|---|---|---|
| Wheelbase | 1.430 m | chassis team reply: 1.445 m |
| Track | 0.970 m | top-view drawing: 0.990 m |
| Wheel radius | 0.2794 m (22 in tyre) | loaded radius |
| Max road-wheel angle | 30° (0.5236 rad) | VCU interface allows 42° |
| Skid-steer (turn on the spot) | not offered | availability on Luchs B |

## Unresolved VCU current and release status

The DBC preserves the source current field as unsigned 10-bit with factor 1 and offset 0, while its source range is described as -200…200 A and marked TO CONFIRM. The SDK currently does not decode this current field. No encoding, sign or offset has been invented for this preview. Obtain signedness, factor/offset, charge/discharge convention and raw example frames from the manufacturer before adding or claiming that feature.

The listed geometry and command defaults remain assumptions, including the distinction between a 30° chassis steering limit and a 42° VCU wire range. Dedicated LCI CAN/pinout/termination, IMU installation, fault-bit meanings, supported motion models and actual heartbeat/stop behavior require firmware-specific confirmation. A simulator or codec unit-test result cannot establish these hardware properties.
