# luchs_sdk

> **Development preview 1.1.0-preview.1.** Adapted from the publicly distributed Luchs software v1.1.0 on 2026-10-11. Code/package versions remain 1.1.0 for source traceability. Tests supplied with the source are not evidence of successful execution or vehicle validation. Luchs B LCI is a draft interface; B configuration and firmware compatibility still require confirmation. See the [version development manual](../../docs/luchs-development-manual.md).

ROS-independent C++17 library for Luchs chassis (Luchs A and Luchs B). The same library is used by the ROS 2 driver (`luchs_ros2`) and the ROS 1 driver (`luchs_ros1`). Builds inside a colcon workspace or on its own:

```bash
cmake -S luchs_sdk -B build && cmake --build build -j && sudo cmake --install build
```

| Header | Content |
|---|---|
| `luchs_sdk/chassis.hpp` | `Chassis`: thread-safe client — control loop, command and feedback watchdogs, ramps, limits, stop on shutdown |
| `luchs_sdk/legacy_a_protocol.hpp` | encode / decode of every Luchs A frame; firmware-dependent details in `legacy_a::Config` |
| `luchs_sdk/lci_protocol.hpp` | Luchs Chassis Interface v1.0 (Luchs B first): commands, heartbeat with checksum, live limits, IMU, brake / EPS, version |
| `luchs_sdk/legacy_b_protocol.hpp` | Luchs B VCU upper interface V2.6 (0x701 / 0x703) and its host-side speed loop |
| `luchs_sdk/kinematics.hpp` | steering solutions for all motion models, limits, odometry |
| `luchs_sdk/socketcan.hpp` | SocketCAN transport, `Transport` interface for other back-ends |
| `luchs_sdk/sim_bus.hpp` | `SimBus` (Luchs A), `SimBusLci`, `SimBusLegacyB`, `make_simulator(options)`: protocol-level chassis simulators for development without hardware |

## Example

```cpp
#include <luchs_sdk/chassis.hpp>
#include <thread>

int main() {
  luchs_sdk::ChassisOptions opt;                       // A-4WS on can0, 20 Hz, 0.5 s watchdog
  opt.geometry = luchs_sdk::Geometry::for_build(luchs_sdk::Build::kFourWheelSteer);
  luchs_sdk::Chassis chassis(opt);                     // or Chassis(opt, std::make_unique<luchs_sdk::SimBus>())
  if (!chassis.start()) { std::fprintf(stderr, "%s\n", chassis.last_error().c_str()); return 1; }

  for (int i = 0; i < 100; ++i) {                      // 2 s at 50 Hz
    chassis.set_twist({0.3, 0.0, 0.2});                // vx m/s, vy m/s, yaw rate rad/s
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  auto s = chassis.state();                            // latest feedback, with timestamps
  std::printf("vx %.2f m/s, battery %.0f %%\n", s.motion.twist.vx, s.battery.soc_percent);
  chassis.stop();                                      // zero + gear P
}
```

Luchs B (LCI, 50 Hz, heartbeat, live limits):

```cpp
auto opt = luchs_sdk::ChassisOptions::for_build(luchs_sdk::Build::kLuchsB);
luchs_sdk::Chassis chassis(opt);                       // or Chassis(opt, luchs_sdk::make_simulator(opt))
chassis.set_speed_limit(3.0);                          // host cap in the heartbeat, m/s
chassis.set_twist({1.0, 0.0, 0.2});                    // dual Ackermann at base_link
auto s = chassis.state();                              // s.limits, s.status, s.imu, s.brake, s.version
```

CMake: `find_package(luchs_sdk REQUIRED)` and `target_link_libraries(app luchs_sdk::luchs_sdk)`.

## luchs_cli

`dump`, `monitor [--raw]`, `encode`, `drive --yes`, `version`. See the repository README.

## Tests

`colcon test --packages-select luchs_sdk` — 63 test cases supplied (execution status is tracked separately): protocol byte vectors (Luchs A, LCI, VCU V2.6), kinematics, watchdogs, heartbeat, live limits, closed loop against all three simulators. `python3 dbc/check_lci_dbc.py` checks the LCI DBC against the same vectors.
