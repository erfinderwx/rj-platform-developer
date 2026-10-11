// Copyright 2026 TerraAgent Robotics. Chassis: RJ Tech Limited.
// SPDX-License-Identifier: Apache-2.0

#include "luchs_sdk/legacy_b_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace luchs_sdk::legacy_b
{
namespace
{
constexpr double kRadToDeg = 57.29577951308232;
uint8_t pct(double v) {return static_cast<uint8_t>(std::clamp(std::round(v), 0.0, 100.0));}
}  // namespace

CanFrame encode_command(const Pedals & p, double angle_rad, double cap_mps, Gear gear,
  const HostRequest & req, const LampCommand & lamps, uint8_t heartbeat, const Config & cfg)
{
  CanFrame f;
  f.id = cfg.cmd_id;
  f.dlc = 8;
  f.data.fill(0);
  auto & d = f.data;
  d[0] = pct(p.throttle);
  d[1] = pct(p.brake);
  const double deg = std::clamp(cfg.steer_sign * angle_rad * kRadToDeg,
      -cfg.max_wheel_angle_deg, cfg.max_wheel_angle_deg);
  d[2] = static_cast<uint8_t>(static_cast<int8_t>(std::lround(deg)));
  const double cap = std::clamp(cap_mps * 3.6, 0.0, std::min(cfg.speed_cap_kmh, 255.0));
  d[3] = static_cast<uint8_t>(std::ceil(cap - 1e-9));
  const auto rate = static_cast<uint16_t>(std::clamp(std::round(cfg.steer_rate_dps), 50.0, 520.0));
  d[4] = static_cast<uint8_t>(rate & 0xFF);
  d[5] = static_cast<uint8_t>((rate >> 8) & 0x03);
  uint8_t gear_wire = 0;   // N
  if (gear == Gear::kDrive) {gear_wire = 1;} else if (gear == Gear::kReverse) {gear_wire = 2;}
  const bool park = req.park || gear == Gear::kPark;
  d[6] = static_cast<uint8_t>((cfg.remote_enable ? 0x01 : 0) | (lamps.headlight ? 0x02 : 0) |
    (req.estop ? 0x04 : 0) | (park ? 0x08 : 0) | (gear_wire << 4));
  d[7] = heartbeat;
  return f;
}

bool decode(const CanFrame & f, const Config & cfg, ChassisState & s, Clock::time_point stamp)
{
  if (f.id != cfg.fb_id || f.dlc < 8) {
    ++s.frames_unknown;
    return false;
  }
  const auto & d = f.data;
  s.battery.raw = {d[0], d[1], 0, d[2]};
  s.battery.soc_percent = d[0];
  s.battery.temperature_c = d[1] - 40.0;
  s.battery.voltage_v = d[2];
  s.battery_stamp = stamp;
  s.status.fault_level = d[3] & 0x03;
  const uint8_t gear = (d[5] >> 2) & 0x03;   // 0 N, 1 D, 2 R
  s.steer.gear = gear == 1 ? Gear::kDrive : gear == 2 ? Gear::kReverse : Gear::kNeutral;
  s.steer.gear_raw = static_cast<uint8_t>(s.steer.gear);   // SDK / LCI numbering (D = 4)
  const bool remote_mode = d[5] & 0x10;      // 1 = upper computer
  s.motion.source = remote_mode ? ControlSource::kHost : ControlSource::kRemoteControl;
  s.motion.control_raw = remote_mode ? 0x22 : 0x11;
  const double speed = d[6] / 3.6;
  s.motion.twist.vx = gear == 2 ? -speed : speed;
  s.motion.twist.vy = 0.0;
  s.motion_stamp = stamp;
  s.status.alive_counter = d[7];
  s.status.operating_mode = remote_mode ? 2 : 1;
  s.status_stamp = stamp;
  ++s.frames_received;
  return true;
}

std::string fault_text(uint8_t level)
{
  switch (level & 0x03) {
    case 0: return "no fault";
    case 1: return "motor controller fault";
    case 2: return "battery (BMS) fault";
    default: return "several faults";
  }
}

std::string describe(const CanFrame & f, const Config & cfg)
{
  char hex[3 * 8 + 1] = {0};
  for (std::size_t i = 0; i < f.dlc && i < 8; ++i) {
    std::snprintf(hex + 3 * i, 4, "%02X ", f.data[i]);
  }
  const char * name = f.id == cfg.cmd_id ? "UpperCmd" : f.id == cfg.fb_id ? "VCU2Upper" : "unknown";
  char out[96];
  std::snprintf(out, sizeof(out), "0x%03X %-14s [%u] %s", f.id, name, f.dlc, hex);
  return out;
}

Pedals SpeedController::step(double target, double measured, double dt, const Config & cfg)
{
  Pedals p;
  const double t = std::abs(target);
  const double m = std::abs(measured);
  if (t < 0.05) {
    integral_ = 0.0;
    p.brake = m < 0.3 ? cfg.hold_brake : std::min(cfg.max_brake, cfg.hold_brake + cfg.brake_kp * m);
    return p;
  }
  const double err = t - m;
  if (dt > 0.0 && dt < 1.0) {
    integral_ = std::clamp(integral_ + err * dt, 0.0,
        cfg.throttle_ki > 0.0 ? cfg.max_throttle / cfg.throttle_ki : 0.0);
  }
  p.throttle = std::clamp(cfg.throttle_ff * t + cfg.throttle_kp * err + cfg.throttle_ki * integral_,
      0.0, cfg.max_throttle);
  if (-err > cfg.overspeed_deadband) {
    p.throttle = 0.0;
    p.brake = std::clamp(cfg.brake_kp * (-err - cfg.overspeed_deadband), 0.0, cfg.max_brake);
  }
  return p;
}

}  // namespace luchs_sdk::legacy_b
