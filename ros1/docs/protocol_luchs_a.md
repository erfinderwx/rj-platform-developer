# Luchs A CAN protocol (v1.1) — reference for the driver

> ROS 1 (`luchs_ros1`): parameter names use `/` where the ROS 2 driver uses `.` (`protocol/type`, `lci/follow_live_limits`); launch files are `luchs_a.launch` / `luchs_b.launch`.

CAN 2.0, **1 Mbit/s**, 11-bit identifiers, DLC 8, **big-endian**, signed values in two's complement. Physical value = raw × factor. The spreadsheet `Luchs_A_CAN_Protocol_v1.1.xlsx` and `dbc/luchs_a_can_v1.1.dbc` describe the same frames.

## Host → chassis (sent by the driver every control cycle)

| ID | Name | Bytes |
|---|---|---|
| 0x5A0 | Motion command | 0–1 vx (int16, 0.001 m/s) · 2–3 vy (int16, 0.001 m/s, 4WS model) · 4–5 yaw rate (int16, 0.001 rad/s) · 6 motion model (1 differential, 2 front Ackermann, 3 four-wheel steer, 4 dual Ackermann) · 7 control mode (0x00 host) |
| 0x5A1 | Gear + axle angles | 0 gear (0 disabled, 1 P, 2 R, 3 N, 4 D) · 1–2 front angle (int16, 0.001 rad) · 3–4 rear angle (int16, 0.001 rad) |
| 0x5A2 / 0x5A3 | Wheel angles (4WS) and lamps | see *Firmware-dependent settings* |
| 0x5A4 | Host time | 0–7 Unix time in ms, uint64 |

## Chassis → host

| ID | Name | Bytes |
|---|---|---|
| 0x5B5 | Motion feedback | as 0x5A0; byte 7: 11 remote control, 22 host |
| 0x5B6 | Gear + axle angles | as 0x5A1 |
| 0x5B7 | Wheel angles (4WS) | FL, RL, FR, RR, int16, 0.001 rad |
| 0x5B8 | Protection | 0 ultrasonic trigger (0 none, 1 left, 2 right, 3 front, 4 rear) · 1 bumper (0, 1 front, 2 rear) · 2 e-stop (0, 1 left, 2 right) · 3 fault code · 4–7 ultrasonic distance front, rear, left, right (uint8) |
| 0x5B9 | Wheel speeds | four int16 |
| 0x5BA | Motor fault words | 0–3 high word, 4–7 low word (uint32) |
| 0x5BB | Battery | 0–1 state of charge · 2–3 temperature (signed) · 4–5 charging progress · 6–7 voltage |
| 0x5BC | Lamps | 0 turn signal · 1 reverse lamp · 2 fault lamp · 3 light/rain sensor (0x00 dark-dry, 0x01 dark-rain, 0x10 light-dry, 0x11 light-rain; sensor builds) |

### Fault codes (0x5B8 byte 3)

0x00 none · 0x01 ultrasonic stop · 0x02 emergency stop · 0x03 parked · 0x04 charging · 0x05–0x08 drive motor FL, FR, RL, RR · 0x09–0x0C steering motor FL, FR, RL, RR · 0x0D remote-control link lost · 0x0E watchdog.

## Firmware-dependent settings

These are driver parameters (`protocol/*`). Check them once per vehicle with the wheels lifted:

| Parameter | Default | How to check |
|---|---|---|
| `wheel_steer_cmd_id` | 1443 (0x5A3, lamps on 0x5A2) | 4WS: `luchs_cli drive --build 4ws --vy 0.1 --seconds 2 --yes` — all four wheels must turn to 90° and the vehicle must move left. If the turn signal blinks instead, set 1442. |
| `wheel_steer_cmd_order` | `lf_rf_lr_rr` | same test with `--vx 0.2 --wz 0.4`: front wheels steer one way, rear wheels the other |
| `wheel_steer_fb_order` | `lf_lr_rf_rr` | `luchs_cli monitor` while steering: FL/FR show the front angles |
| `wheel_speed_fb_order` | `lf_lr_rf_rr` | lift one wheel and turn it by hand: the matching slot changes |
| `wheel_speed_scale` | 0.001 (raw → rad/s) | drive at 0.3 m/s: wheel speed ≈ 0.3 / 0.103 = 2.9 rad/s |
| `ultrasonic_scale` | 0.01 (raw → m) | hold a board 0.50 m in front of the sensor |
| `battery_soc_scale` | 0.1 (raw → %) | compare with the charge display on the chassis |
| `battery_temperature_scale` | 0.1 (raw → °C) | plausible room temperature when cold |
| `battery_voltage_scale` | 0.1 (raw → V) | 48 V LiFePO4: about 50–54 V when charged |

`luchs_cli monitor --raw` shows the raw values next to the scaled ones. Write the confirmed values into your parameter file.

## Behaviour on the bus

- The chassis acts on the latest 0x5A0 / 0x5A1. The driver sends a complete set every cycle (default 20 Hz) and a stop set (zero, gear P) on shutdown.
- With the mode selector on remote control, the chassis ignores host commands and reports control mode 11.
