// Copyright 2026 TerraAgent Robotics. Chassis: RJ Tech Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Codec for the Luchs Chassis Interface (LCI) v1.0, draft r3: the CAN interface shared by the
// Luchs family. Luchs B is the first chassis to implement it.
//
//   Physical layer : dedicated customer CAN 2.0, 1 Mbit/s, 11-bit identifiers, DLC 8
//   Byte order     : big-endian (Motorola), signed values in two's complement
//   Reference point: rear-axle centre on the ground, X forward, Y left, Z up (REP 103)
//   Host -> chassis: 0x5A0 motion, 0x5A1 gear + axle angles, 0x5A3 lamps / horn,
//                    0x5A4 host time, 0x5A5 heartbeat (20 ms, 200 ms timeout, checksum)
//   Chassis -> host: 0x5B0 live limits, 0x5B1 IMU accel, 0x5B2 attitude, 0x5B5 motion,
//                    0x5B6 gear + axle angles, 0x5B8 protection, 0x5B9 wheel speeds,
//                    0x5BA motor faults, 0x5BB battery, 0x5BD chassis state (checksum),
//                    0x5BE IMU rates, 0x5BF version, 0x5C0 brake / EPB / EPS (B),
//                    0x5C1 motor temperatures (B)
//
// The SDK API works at base_link (centre of the wheelbase). This codec converts commands and
// feedback between base_link and the LCI reference point, using Config::reference_x.
// Layouts follow Luchs_B_LCI_v1.0_draft.dbc (r3); dbc/check_lci_dbc.py checks them with cantools.

#ifndef LUCHS_SDK__LCI_PROTOCOL_HPP_
#define LUCHS_SDK__LCI_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "luchs_sdk/types.hpp"

namespace luchs_sdk::lci
{

// ---- identifiers --------------------------------------------------------------------------
constexpr uint32_t kMotionCmd = 0x5A0;
constexpr uint32_t kSteerCmd = 0x5A1;
constexpr uint32_t kLampCmd = 0x5A3;
constexpr uint32_t kHostTime = 0x5A4;
constexpr uint32_t kHeartbeat = 0x5A5;

constexpr uint32_t kLimitsFb = 0x5B0;
constexpr uint32_t kImuAccelFb = 0x5B1;
constexpr uint32_t kAttitudeFb = 0x5B2;
constexpr uint32_t kMotionFb = 0x5B5;
constexpr uint32_t kSteerFb = 0x5B6;
constexpr uint32_t kProtectionFb = 0x5B8;
constexpr uint32_t kWheelSpeedFb = 0x5B9;
constexpr uint32_t kMotorFaultFb = 0x5BA;
constexpr uint32_t kBatteryFb = 0x5BB;
constexpr uint32_t kChassisStateFb = 0x5BD;
constexpr uint32_t kImuRateFb = 0x5BE;
constexpr uint32_t kVersionFb = 0x5BF;
constexpr uint32_t kBrakeSteerFb = 0x5C0;
constexpr uint32_t kMotorTempFb = 0x5C1;

constexpr uint32_t kBitrate = 1000000;
constexpr uint8_t kLciMajor = 1;
constexpr uint8_t kLciMinor = 0;

/// 0x5BF ModelId values.
constexpr uint8_t kModelA_DF = 0x0A;
constexpr uint8_t kModelA_FA = 0x0B;
constexpr uint8_t kModelA_DA = 0x0C;
constexpr uint8_t kModelA_4WS = 0x0D;
constexpr uint8_t kModelB = 0x1B;
constexpr uint8_t kModelC = 0x2C;
uint8_t model_id_for(Build b);

/// 0x5A1 SteerSource.
enum class SteerSource : uint8_t
{
  kYawRate = 0,   ///< chassis steers from TargetWz in 0x5A0
  kAngle = 1,     ///< chassis steers to the angles in 0x5A1 (also at standstill)
};

struct Config
{
  /// x of the LCI reference point (rear-axle centre) in base_link, metres: -wheelbase / 2.
  /// The Chassis client sets it from its geometry; the default is the Luchs B value.
  double reference_x{-0.715};
  /// 0x5A1 AccelLimit, m/s^2. 0 = chassis default ramp.
  double accel_limit{0.0};
  /// Ignore 0x5BD frames whose checksum is wrong (they are counted in Status::checksum_errors).
  bool check_checksum{true};
};

// ---- encoding (host -> chassis) -----------------------------------------------------------
/// `twist` at base_link (SDK convention); converted to the reference point here.
CanFrame encode_motion(const Twist2D & twist, MotionModel model, const Config & cfg);
CanFrame encode_steer(Gear gear, double front_rad, double rear_rad, SteerSource source,
  double accel_limit);
CanFrame encode_lamps(const LampCommand & lamps);
CanFrame encode_host_time(uint64_t unix_ms);
CanFrame encode_heartbeat(uint8_t alive_counter, const HostRequest & req);

/// Low byte of the sum of bytes 0-6, XOR 0xFF (0x5A5 and 0x5BD).
uint8_t checksum(const std::array<uint8_t, 8> & d);

/// All frames for one control cycle, in transmission order: 0x5A0, 0x5A1, 0x5A5 every cycle,
/// 0x5A3 when `lamps` is given (every 100 ms), 0x5A4 when `unix_ms` is given (every 1 s).
std::vector<CanFrame> encode_cycle(
  const MotionCommand & cmd, double front_rad, double rear_rad, SteerSource source,
  uint8_t alive_counter, const HostRequest & req, const Config & cfg,
  const std::optional<LampCommand> & lamps = std::nullopt,
  std::optional<uint64_t> unix_ms = std::nullopt);

// ---- decoding (chassis -> host) -----------------------------------------------------------
/// Decodes one frame into `state` (motion converted to base_link). Returns false for
/// identifiers that are not LCI feedback, for short frames and for 0x5BD frames with a wrong
/// checksum.
bool decode(const CanFrame & frame, const Config & cfg, ChassisState & state,
  Clock::time_point stamp = Clock::now());

std::string describe(const CanFrame & frame);   ///< one-line text, for logs and the CLI

}  // namespace luchs_sdk::lci

#endif  // LUCHS_SDK__LCI_PROTOCOL_HPP_
