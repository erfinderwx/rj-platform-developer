// Copyright 2026 TerraAgent Robotics. Chassis: RJ Tech Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Plain data types shared by the Luchs SDK (Luchs A and Luchs B). No ROS, no I/O.

#ifndef LUCHS_SDK__TYPES_HPP_
#define LUCHS_SDK__TYPES_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace luchs_sdk
{

constexpr const char * kSdkVersion = "1.1.0";

using Clock = std::chrono::steady_clock;

/// One classic CAN 2.0 frame (11-bit standard identifier, up to 8 data bytes).
struct CanFrame
{
  uint32_t id{0};
  uint8_t dlc{8};
  std::array<uint8_t, 8> data{};
};

/// Mechanical build of a Luchs chassis. Fixed by the hardware, set once in the configuration.
enum class Build : uint8_t
{
  kSkidSteer = 0,      ///< A-DF  four-wheel drive, no steering (differential / skid-steer)
  kFrontAckermann,     ///< A-FA  front-axle Ackermann steering
  kDualAckermann,      ///< A-DA  front- and rear-axle Ackermann steering
  kFourWheelSteer,     ///< A-4WS four independently steered wheels
  kLuchsB,             ///< B     four hub motors, front + rear EPS: single or dual Ackermann
};

/// CAN protocol spoken on the host bus.
enum class Protocol : uint8_t
{
  kLegacyA = 0,   ///< Luchs A chassis protocol v1.1 (units shipping today)
  kLci,           ///< Luchs Chassis Interface v1.0 (draft r3): the family interface, Luchs B first
  kLegacyB,       ///< Luchs B VCU upper-computer interface V2.6 (0x701 / 0x703), units shipping today
};
const char * protocol_name(Protocol p);
const char * build_name(Build b);

/// Kinematic model requested from the chassis firmware (CAN 0x5A0 byte 6).
/// Values are the wire values of the Luchs A protocol v1.1.
enum class MotionModel : uint8_t
{
  kDifferential = 0x01,
  kFrontAckermann = 0x02,
  kFourWheelSteer = 0x03,   ///< omnidirectional: vx, vy and yaw rate together
  kDualAckermann = 0x04,
};

/// Gear (CAN 0x5A1 / 0x5B6 byte 0).
enum class Gear : uint8_t
{
  kDisabled = 0, kPark = 1, kReverse = 2, kNeutral = 3, kDrive = 4,
};

/// Who is driving, as reported by the chassis (0x5B5 byte 7).
enum class ControlSource : uint8_t
{
  kUnknown = 0, kRemoteControl, kHost,
};

enum class TurnSignal : uint8_t { kOff = 0, kLeft = 1, kRight = 2, kHazard = 3 };   ///< hazard: LCI only

/// Wheel index order used everywhere in the SDK API (independent of the wire order).
enum Wheel : std::size_t { kFrontLeft = 0, kFrontRight = 1, kRearLeft = 2, kRearRight = 3 };
using PerWheel = std::array<double, 4>;

/// Body twist in the chassis frame (x forward, y left, z up). SI units.
struct Twist2D
{
  double vx{0.0};   ///< m/s
  double vy{0.0};   ///< m/s  (four-wheel-steer build only)
  double wz{0.0};   ///< rad/s
};

/// What the host wants the chassis to do. The SDK fills the steering fields from the twist
/// (see Kinematics) unless explicit angles are given.
struct MotionCommand
{
  Twist2D twist;
  MotionModel model{MotionModel::kDifferential};
  Gear gear{Gear::kDrive};
  std::optional<double> front_steer_rad;   ///< Ackermann builds; computed from twist if empty
  std::optional<double> rear_steer_rad;
  std::optional<PerWheel> wheel_steer_rad; ///< 4WS build; computed from twist if empty
};

struct LampCommand
{
  TurnSignal turn{TurnSignal::kOff};
  bool reverse{false};
  bool fault{false};
  uint8_t headlight{0};   ///< 0 off, 1 low, 2 high (LCI; legacy B: on/off)
  bool horn{false};       ///< LCI
  bool beacon{false};     ///< LCI, where fitted
};

/// Requests carried by the host heartbeat (LCI 0x5A5; legacy B 0x701). Ignored by legacy A.
struct HostRequest
{
  bool estop{false};            ///< emergency stop, latched by the chassis until a maintenance reset
  bool park{false};             ///< controlled stop and hold (parking brake)
  bool payload_power_1{false};  ///< switched payload output 1, where fitted
  bool payload_power_2{false};  ///< switched payload output 2, where fitted
  double speed_limit{0.0};      ///< m/s, host-side cap; 0 = chassis default
};

/// Latest decoded feedback from the chassis. Every block carries the time it was received,
/// so a consumer can tell a fresh value from a stale one.
struct ChassisState
{
  struct Motion
  {
    Twist2D twist;
    uint8_t model_raw{0};
    uint8_t control_raw{0};
    ControlSource source{ControlSource::kUnknown};
  };
  struct Steer
  {
    Gear gear{Gear::kDisabled};
    uint8_t gear_raw{0};
    double front_rad{0.0};
    double rear_rad{0.0};
  };
  struct Safety
  {
    uint8_t ultrasonic_trigger_raw{0};  ///< 0 none, 1 left, 2 right, 3 front, 4 rear
    uint8_t bumper_raw{0};              ///< 0 none, 1 front, 2 rear
    uint8_t estop_raw{0};               ///< 0 none, 1 left, 2 right
    uint8_t fault_code{0};              ///< see fault_code_text()
    std::array<double, 4> ultrasonic_m{};   ///< front, rear, left, right (scaled, see Config)
    std::array<uint8_t, 4> ultrasonic_raw{};
  };
  struct Battery
  {
    double soc_percent{0.0};
    double temperature_c{0.0};
    double voltage_v{0.0};
    double charging_percent{0.0};
    std::array<uint16_t, 4> raw{};      ///< soc, temperature, charging, voltage as received
  };
  struct Lamps
  {
    uint8_t turn_raw{0};
    uint8_t reverse_raw{0};
    uint8_t fault_raw{0};
    uint8_t light_rain_raw{0};          ///< 0x00 dark/dry, 0x01 dark/rain, 0x10 light/dry, 0x11 light/rain
  };

  // ---- LCI only (Luchs B first). Empty stamps on chassis that do not send them. ----------
  struct Limits                         ///< 0x5B0: what the chassis enforces right now
  {
    double max_vx_fwd{0.0};             ///< m/s
    double max_vx_rev{0.0};             ///< m/s
    double max_steer_rad{0.0};          ///< |road-wheel angle| allowed at the current speed
    uint8_t reason_bits{0};             ///< see limit_reason_text()
    bool speed_clipped{false}, steer_clipped{false}, accel_clipped{false};
  };
  struct Imu                            ///< 0x5B1 / 0x5BE / 0x5B2, vehicle frame (X fwd, Y left, Z up)
  {
    std::array<double, 3> accel{};      ///< m/s^2, gravity included
    std::array<double, 3> rate{};       ///< rad/s
    double roll{0.0}, pitch{0.0};       ///< rad, chassis estimate (+right down, +nose down)
    uint8_t accel_counter{0}, rate_counter{0};
    uint8_t status{0};                  ///< 0 ok, 1 initialising, 2 fault
  };
  struct Status                         ///< 0x5BD: who is in control, why it stopped
  {
    uint8_t operating_mode{0};          ///< 0 maintenance, 1 remote control, 2 automatic
    uint8_t estop_source_bits{0};       ///< see estop_source_text()
    uint8_t fault_level{0};             ///< 0 none, 1 warning, 2 derated, 3 stop
    bool rc_online{false}, rc_enabled{false}, four_wd{false}, park_brake{false};
    uint8_t rc_link{3};                 ///< 0 good, 1 weak, 2 lost, 3 not fitted
    bool cmd_clipped{false};
    double active_speed_limit{0.0};     ///< m/s
    uint8_t alive_counter{0};
    uint64_t checksum_errors{0};        ///< frames with a wrong checksum (frame ignored)
    uint64_t counter_errors{0};         ///< AliveCounter did not advance by one
  };
  struct BrakeSteer                     ///< 0x5C0 (Luchs B)
  {
    double brake_pressure_mpa{0.0};
    uint8_t epb_state{0};               ///< 0 released, 1 applying, 2 applied, 3 releasing, 4 fault
    uint8_t eps_front{0}, eps_rear{0}, brake_state{0};   ///< 0 ok, 1 warning, 2 degraded, 3 failed
  };
  struct Temperatures                   ///< 0x5C1 (Luchs B), deg C, LF RF LR RR
  {
    std::array<double, 4> motor{};
    std::array<double, 4> controller{};
  };
  struct Version                        ///< 0x5BF
  {
    uint8_t lci_major{0}, lci_minor{0}, model_id{0}, fw_major{0}, fw_minor{0}, fw_patch{0};
  };

  std::optional<Clock::time_point> motion_stamp, steer_stamp, wheel_steer_stamp, safety_stamp,
    wheel_speed_stamp, motor_fault_stamp, battery_stamp, lamps_stamp;
  std::optional<Clock::time_point> limits_stamp, imu_accel_stamp, imu_rate_stamp, attitude_stamp,
    status_stamp, brake_stamp, temperature_stamp, version_stamp;

  Motion motion;
  Steer steer;
  PerWheel wheel_steer_rad{};         ///< 4WS build
  PerWheel wheel_speed{};             ///< scaled per Config::wheel_speed_scale (rad/s by default)
  std::array<int16_t, 4> wheel_speed_raw{};
  uint32_t motor_fault_high{0};
  uint32_t motor_fault_low{0};
  Safety safety;
  Battery battery;
  Lamps lamps;
  Limits limits;
  Imu imu;
  Status status;
  BrakeSteer brake;
  Temperatures temperature;
  Version version;

  uint64_t frames_received{0};
  uint64_t frames_unknown{0};
};

/// Human-readable text for the chassis fault code (0x5B8 byte 3).
std::string fault_code_text(uint8_t code);
/// LCI texts: 0x5B0 LimitReason bits, 0x5BD EStopSource bits, 0x5BF ModelId.
std::string limit_reason_text(uint8_t bits);
std::string estop_source_text(uint8_t bits);
std::string model_id_text(uint8_t id);

}  // namespace luchs_sdk

#endif  // LUCHS_SDK__TYPES_HPP_
