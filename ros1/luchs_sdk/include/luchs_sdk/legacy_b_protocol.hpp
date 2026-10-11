// Copyright 2026 TerraAgent Robotics. Chassis: RJ Tech Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Codec for the Luchs B VCU upper-computer interface V2.6 ("legacy B"): the interface of the
// Luchs B units shipping today, until their firmware implements LCI.
//
//   Host -> chassis: 0x701 UpperCmd, 10 ms: throttle and brake pedal (%), road-wheel angle (deg),
//                    speed cap (km/h), steering rate, remote enable, headlight, e-stop, park,
//                    gear (N/D/R), heartbeat
//   Chassis -> host: 0x703 VCU2Upper, 10 ms: SOC, battery temperature and voltage, fault level,
//                    gear, mode, speed (km/h, unsigned), heartbeat
//   Byte order     : little-endian (Intel)
//
// This interface is pedal-level: the host closes the speed loop (SpeedController below) and the
// chassis reports speed in whole km/h, with no yaw rate and no steering feedback. It is good for
// bring-up, teleoperation and data recording; for autonomy use LCI (lci_protocol.hpp).
// Items marked TO CONFIRM are parameters, so a vehicle can be matched without rebuilding.

#ifndef LUCHS_SDK__LEGACY_B_PROTOCOL_HPP_
#define LUCHS_SDK__LEGACY_B_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <string>

#include "luchs_sdk/types.hpp"

namespace luchs_sdk::legacy_b
{

struct Config
{
  // The protocol sheet writes "701" / "703" without a radix. Default: hexadecimal. TO CONFIRM.
  uint32_t cmd_id{0x701};
  uint32_t fb_id{0x703};
  /// Sign of the road-wheel angle on the wire relative to ROS (+left). The EPS sheet counts
  /// clockwise (steer right) as positive, so the default is -1. TO CONFIRM with wheels lifted.
  int steer_sign{-1};
  double max_wheel_angle_deg{42.0};   ///< wire range [-42, 42]
  double steer_rate_dps{200.0};       ///< wire range [50, 520] deg/s
  double speed_cap_kmh{30.0};         ///< wire range [0, 30]
  bool remote_enable{true};           ///< bit 48: 1 = upper computer (host) control

  // Speed loop closed by the host (pedal interface).
  double throttle_ff{10.0};           ///< % per m/s of target speed
  double throttle_kp{20.0};           ///< % per m/s of speed error
  double throttle_ki{8.0};            ///< % per (m/s * s)
  double max_throttle{40.0};          ///< %
  double brake_kp{40.0};              ///< % per m/s of overspeed
  double overspeed_deadband{0.4};     ///< m/s; the speed feedback has 1 km/h resolution
  double hold_brake{30.0};            ///< % while stopped
  double max_brake{80.0};             ///< %
};

/// Pedal positions sent in 0x701.
struct Pedals
{
  double throttle{0.0};   ///< %
  double brake{0.0};      ///< %
};

CanFrame encode_command(const Pedals & pedals, double wheel_angle_rad, double speed_cap_mps,
  Gear gear, const HostRequest & req, const LampCommand & lamps, uint8_t heartbeat,
  const Config & cfg);

/// Decodes 0x703 into `state`: battery, gear, control source, speed (signed by the gear;
/// motion.twist.wz is left to the caller, it needs the commanded angle). Returns false for
/// other identifiers and short frames.
bool decode(const CanFrame & frame, const Config & cfg, ChassisState & state,
  Clock::time_point stamp = Clock::now());

/// Fault level in 0x703: 0 none, 1 motor controller, 2 BMS, 3 several.
std::string fault_text(uint8_t level);

std::string describe(const CanFrame & frame, const Config & cfg);

/// PI speed loop on pedal positions with feed-forward; brakes on overspeed and holds at rest.
class SpeedController
{
public:
  Pedals step(double target_speed, double measured_speed, double dt, const Config & cfg);
  void reset() {integral_ = 0.0;}

private:
  double integral_{0.0};
};

}  // namespace luchs_sdk::legacy_b

#endif  // LUCHS_SDK__LEGACY_B_PROTOCOL_HPP_
