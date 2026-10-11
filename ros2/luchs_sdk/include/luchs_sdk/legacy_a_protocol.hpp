// Copyright 2026 TerraAgent Robotics. Chassis: RJ Tech Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Codec for the Luchs A chassis CAN protocol v1.1 ("legacy A" protocol).
//
//   Physical layer : CAN 2.0, 1 Mbit/s, 11-bit standard identifiers, DLC 8
//   Byte order     : big-endian (high byte first), signed values in two's complement
//   Host -> chassis: 0x5A0 motion, 0x5A1 gear + Ackermann angles, 0x5A2 / 0x5A3 wheel angles and
//                    lamps (see Config::wheel_steer_cmd_id), 0x5A4 host time
//   Chassis -> host: 0x5B5 motion, 0x5B6 gear + Ackermann angles, 0x5B7 wheel angles,
//                    0x5B8 protection, 0x5B9 wheel speeds, 0x5BA motor faults, 0x5BB battery,
//                    0x5BC lamps
//
// Every value whose scaling or position is not yet confirmed by the chassis firmware team is a
// field of Config, so it can be corrected from a parameter file without rebuilding. The
// defaults reproduce the behaviour of the reference driver that has been run on vehicles.
// `luchs_cli monitor --raw` prints the raw values needed to confirm them on a unit.

#ifndef LUCHS_SDK__LEGACY_A_PROTOCOL_HPP_
#define LUCHS_SDK__LEGACY_A_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "luchs_sdk/types.hpp"

namespace luchs_sdk::legacy_a
{

// ---- identifiers --------------------------------------------------------------------------
constexpr uint32_t kMotionCmd = 0x5A0;
constexpr uint32_t kSteerCmd = 0x5A1;
constexpr uint32_t kId5A2 = 0x5A2;
constexpr uint32_t kId5A3 = 0x5A3;
constexpr uint32_t kHostTimeCmd = 0x5A4;

constexpr uint32_t kMotionFb = 0x5B5;
constexpr uint32_t kSteerFb = 0x5B6;
constexpr uint32_t kWheelSteerFb = 0x5B7;
constexpr uint32_t kProtectionFb = 0x5B8;
constexpr uint32_t kWheelSpeedFb = 0x5B9;
constexpr uint32_t kMotorFaultFb = 0x5BA;
constexpr uint32_t kBatteryFb = 0x5BB;
constexpr uint32_t kLampFb = 0x5BC;

constexpr uint32_t kBitrate = 1000000;

/// Order in which the four wheels appear in a frame.
enum class WheelOrder : uint8_t
{
  kLfRfLrRr,   ///< front-left, front-right, rear-left, rear-right
  kLfLrRfRr,   ///< front-left, rear-left, front-right, rear-right
};

struct Config
{
  // -- Wheel-angle command (4WS build) ------------------------------------------------------
  // The protocol document assigns 0x5A2 to wheel angles (order LF, LR, RF, RR) and 0x5A3 to
  // lamps. The reference driver run on vehicles sends wheel angles on 0x5A3 in the order
  // LF, RF, LR, RR and lamps on 0x5A2. Default = reference driver. TO BE CONFIRMED.
  uint32_t wheel_steer_cmd_id{kId5A3};
  WheelOrder wheel_steer_cmd_order{WheelOrder::kLfRfLrRr};
  // The lamp command uses whichever of 0x5A2 / 0x5A3 is not the wheel-angle command.
  uint32_t lamp_cmd_id() const { return wheel_steer_cmd_id == kId5A2 ? kId5A3 : kId5A2; }

  // -- Feedback layout and scaling (TO BE CONFIRMED where noted) ----------------------------
  WheelOrder wheel_steer_fb_order{WheelOrder::kLfLrRfRr};   ///< 0x5B7: document and driver agree
  WheelOrder wheel_speed_fb_order{WheelOrder::kLfLrRfRr};   ///< 0x5B9: document says LF,RF,LR,RR
  double wheel_speed_scale{0.001};     ///< 0x5B9 raw -> rad/s (unit not published)
  double ultrasonic_scale_m{0.01};     ///< 0x5B8 raw -> metres ("x100" in the document)
  double battery_soc_scale{0.1};       ///< 0x5BB raw -> percent (document: x1000 of a fraction)
  double battery_temp_scale{0.1};      ///< 0x5BB raw (signed) -> deg C
  double battery_voltage_scale{0.1};   ///< 0x5BB raw -> volt (measured on vehicles: 0.1 V/LSB)
  double charging_scale{1.0};          ///< 0x5BB raw -> percent
};

// ---- encoding (host -> chassis) -----------------------------------------------------------
CanFrame encode_motion(const Twist2D & twist, MotionModel model, uint8_t control_mode = 0x00);
CanFrame encode_steer(Gear gear, double front_rad, double rear_rad);
CanFrame encode_wheel_steer(const PerWheel & angles_rad, const Config & cfg);
CanFrame encode_lamps(const LampCommand & lamps, const Config & cfg);
CanFrame encode_host_time(uint64_t unix_ms);

/// All frames for one control cycle, in transmission order.
std::vector<CanFrame> encode_cycle(
  const MotionCommand & cmd, const PerWheel & wheel_steer_rad, double front_rad,
  double rear_rad, const LampCommand & lamps, const Config & cfg,
  std::optional<uint64_t> unix_ms = std::nullopt);

// ---- decoding (chassis -> host) -----------------------------------------------------------
/// Decodes one frame into `state`. Returns false for identifiers that are not chassis
/// feedback (including the host's own commands echoed on the bus) or for short frames.
bool decode(const CanFrame & frame, const Config & cfg, ChassisState & state,
  Clock::time_point stamp = Clock::now());

// ---- helpers ------------------------------------------------------------------------------
/// Physical value -> int16 with rounding and saturation (scale = 1000 for this protocol).
int16_t to_raw16(double value, double scale = 1000.0);
inline int16_t be_i16(uint8_t hi, uint8_t lo)
{
  return static_cast<int16_t>(static_cast<uint16_t>((hi << 8) | lo));
}
inline uint16_t be_u16(uint8_t hi, uint8_t lo) {return static_cast<uint16_t>((hi << 8) | lo);}
inline void put_i16(std::array<uint8_t, 8> & d, std::size_t at, int16_t v)
{
  const auto u = static_cast<uint16_t>(v);
  d[at] = static_cast<uint8_t>(u >> 8);
  d[at + 1] = static_cast<uint8_t>(u & 0xFF);
}

std::string describe(const CanFrame & frame);   ///< one-line text, for logs and the CLI

}  // namespace luchs_sdk::legacy_a

#endif  // LUCHS_SDK__LEGACY_A_PROTOCOL_HPP_
