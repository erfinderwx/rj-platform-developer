# Troubleshooting

> **Development preview 1.1.0-preview.1.** Adapted from the publicly distributed Luchs software v1.1.0 on 2026-10-11. Code/package versions remain 1.1.0 for source traceability. Tests supplied with the source are not evidence of successful execution or vehicle validation. Luchs B LCI is a draft interface; B configuration and firmware compatibility still require confirmation. See the [version development manual](../../docs/luchs-development-manual.md).

| Symptom | Check |
|---|---|
| `cannot open CAN interface 'can0'` | `ip -br link` — interface missing: adapter not detected; present but DOWN: run `setup_can.sh` |
| Diagnostics "no chassis feedback" | chassis powered? `candump can0` empty → wiring, termination, bitrate (must be 1 Mbit/s) |
| Feedback OK, vehicle does not move | mode selector on remote control (`~/state.control_source` = 1); e-stop pressed (`~/safety`); `cmd_vel` arriving? (`ros2 topic hz /cmd_vel`); gear P while idle is normal |
| Moves, then stops after 0.5 s | `cmd_vel` not re-published: teleop must keep publishing, or raise `control.cmd_timeout` |
| `command_limited: true` | request above `limits.*` or beyond the steering range; expected for tight Ackermann turns |
| Odometry drifts in yaw | normal for wheel odometry; fuse an IMU (docs/integration.md) |
| 4WS wheels do not follow, turn signal blinks | `protocol.wheel_steer_cmd_id` — see docs/protocol_luchs_a.md |
| Battery numbers implausible | `protocol.battery_*_scale`, check with `luchs_cli monitor --raw` |
| ERROR-PASSIVE / BUS-OFF | wrong bitrate, missing 120 Ω termination, CAN-H/L swapped |
| **Luchs B (LCI draft/simulator): hardware behavior requires confirmation** | |
| Feedback OK, vehicle does not move, `estop_source_text: host heartbeat timeout` | the draft/simulator reports a 200 ms heartbeat timeout; actual firmware behavior requires confirmation: CPU overload, bus errors, or the driver was stopped. Check `candump can0 | grep 5A5` |
| `estop_source_text: host request` and it stays | an e-stop request (`~/estop`) is latched by the chassis: obtain and follow the manufacturer-approved recovery procedure for the installed firmware |
| `operating_mode: 1` | transmitter switch C up: the remote control has priority |
| `command_limited: true`, `limit_reason_text` set | the chassis is derating (slope, temperature, battery, host speed limit); the driver follows its limits and keeps the path curvature |
| Diagnostics "chassis identifies as …" | `build` does not match the chassis (0x5BF ModelId) |
| `chassis-state checksum errors` | noise or a second node on the bus sending 0x5BD |
| **Luchs B (VCU interface V2.6)** | |
| No feedback | identifiers (`legacy_b.cmd_id` / `fb_id`, the sheet writes "701") and bit rate — see docs/protocol_luchs_b.md |
| Steers the wrong way | `legacy_b.steer_sign` |
| Jerky speed | expected with 1 km/h speed feedback; lower `legacy_b.throttle_kp`, or use LCI firmware |

Collect for support: driver log, `ros2 topic echo --once /diagnostics`, `ros2 param dump /luchs_driver`, `candump -l can0` (10 s).
