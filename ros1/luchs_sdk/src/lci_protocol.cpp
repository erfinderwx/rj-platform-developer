// Copyright 2026 TerraAgent Robotics. Chassis: RJ Tech Limited.
// SPDX-License-Identifier: Apache-2.0

#include "luchs_sdk/lci_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "luchs_sdk/legacy_a_protocol.hpp"   // shared big-endian helpers

namespace luchs_sdk
{

const char * protocol_name(Protocol p)
{
  switch (p) {
    case Protocol::kLegacyA: return "Luchs A protocol v1.1";
    case Protocol::kLci: return "LCI v1.0 (draft r3)";
    case Protocol::kLegacyB: return "Luchs B VCU upper interface V2.6";
  }
  return "?";
}

const char * build_name(Build b)
{
  switch (b) {
    case Build::kSkidSteer: return "A-DF";
    case Build::kFrontAckermann: return "A-FA";
    case Build::kDualAckermann: return "A-DA";
    case Build::kFourWheelSteer: return "A-4WS";
    case Build::kLuchsB: return "B";
  }
  return "?";
}

std::string limit_reason_text(uint8_t bits)
{
  static const char * kNames[8] = {"slope/roll", "temperature", "battery", "fault derate",
    "host speed limit", "speed-dependent steering limit", "maintenance/RC", "other"};
  std::string out;
  for (int i = 0; i < 8; ++i) {
    if (bits & (1u << i)) {
      if (!out.empty()) {out += ", ";}
      out += kNames[i];
    }
  }
  return out.empty() ? "none" : out;
}

std::string estop_source_text(uint8_t bits)
{
  static const char * kNames[8] = {"onboard button left", "onboard button right",
    "remote control", "host request", "front bumper", "rear bumper", "external stop loop",
    "host heartbeat timeout"};
  std::string out;
  for (int i = 0; i < 8; ++i) {
    if (bits & (1u << i)) {
      if (!out.empty()) {out += ", ";}
      out += kNames[i];
    }
  }
  return out.empty() ? "none" : out;
}

std::string model_id_text(uint8_t id)
{
  switch (id) {
    case 0x0A: return "Luchs A-DF";
    case 0x0B: return "Luchs A-FA";
    case 0x0C: return "Luchs A-DA";
    case 0x0D: return "Luchs A-4WS";
    case 0x1B: return "Luchs B";
    case 0x2C: return "Luchs C";
    default: {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "unknown model 0x%02X", id);
        return buf;
      }
  }
}

namespace lci
{
using legacy_a::be_i16;
using legacy_a::be_u16;
using legacy_a::put_i16;
using legacy_a::to_raw16;

namespace
{
CanFrame frame(uint32_t id)
{
  CanFrame f;
  f.id = id;
  f.dlc = 8;
  f.data.fill(0);
  return f;
}
void put_u16(std::array<uint8_t, 8> & d, std::size_t at, double value, double scale)
{
  double r = std::isfinite(value) ? std::round(value * scale) : 0.0;
  r = std::clamp(r, 0.0, 65535.0);
  const auto u = static_cast<uint16_t>(r);
  d[at] = static_cast<uint8_t>(u >> 8);
  d[at + 1] = static_cast<uint8_t>(u & 0xFF);
}
ControlSource source_from_raw(uint8_t raw)
{
  if (raw == 0x11) {return ControlSource::kRemoteControl;}
  if (raw == 0x22) {return ControlSource::kHost;}
  return ControlSource::kUnknown;
}
}  // namespace

uint8_t model_id_for(Build b)
{
  switch (b) {
    case Build::kSkidSteer: return kModelA_DF;
    case Build::kFrontAckermann: return kModelA_FA;
    case Build::kDualAckermann: return kModelA_DA;
    case Build::kFourWheelSteer: return kModelA_4WS;
    case Build::kLuchsB: return kModelB;
  }
  return 0;
}

uint8_t checksum(const std::array<uint8_t, 8> & d)
{
  unsigned sum = 0;
  for (std::size_t i = 0; i < 7; ++i) {sum += d[i];}
  return static_cast<uint8_t>((sum & 0xFFu) ^ 0xFFu);
}

CanFrame encode_motion(const Twist2D & t, MotionModel model, const Config & cfg)
{
  CanFrame f = frame(kMotionCmd);
  // Only the four-wheel-steer model carries a lateral speed; the others send 0 (LCI 0x5A0).
  const double vy = model == MotionModel::kFourWheelSteer ? t.vy + t.wz * cfg.reference_x : 0.0;
  put_i16(f.data, 0, to_raw16(t.vx));
  put_i16(f.data, 2, to_raw16(vy));
  put_i16(f.data, 4, to_raw16(t.wz));
  f.data[6] = static_cast<uint8_t>(model);
  f.data[7] = 0x00;   // ControlMode: autonomous (only value)
  return f;
}

CanFrame encode_steer(Gear gear, double front, double rear, SteerSource src, double accel)
{
  CanFrame f = frame(kSteerCmd);
  f.data[0] = static_cast<uint8_t>(gear);
  put_i16(f.data, 1, to_raw16(front));
  put_i16(f.data, 3, to_raw16(rear));
  f.data[5] = static_cast<uint8_t>(src);
  put_u16(f.data, 6, accel, 1000.0);
  return f;
}

CanFrame encode_lamps(const LampCommand & l)
{
  CanFrame f = frame(kLampCmd);
  f.data[0] = static_cast<uint8_t>(l.turn);
  f.data[1] = l.reverse ? 1 : 0;
  f.data[2] = l.fault ? 1 : 0;
  f.data[3] = std::min<uint8_t>(l.headlight, 2);
  f.data[4] = l.horn ? 1 : 0;
  f.data[5] = l.beacon ? 1 : 0;
  return f;
}

CanFrame encode_host_time(uint64_t ms)
{
  CanFrame f = legacy_a::encode_host_time(ms);
  f.id = kHostTime;
  return f;
}

CanFrame encode_heartbeat(uint8_t counter, const HostRequest & r)
{
  CanFrame f = frame(kHeartbeat);
  f.data[0] = counter;
  f.data[1] = static_cast<uint8_t>((r.estop ? 0x01 : 0) | (r.park ? 0x02 : 0) |
    (r.payload_power_1 ? 0x08 : 0) | (r.payload_power_2 ? 0x10 : 0));
  put_u16(f.data, 2, std::max(0.0, r.speed_limit), 100.0);
  f.data[7] = checksum(f.data);
  return f;
}

std::vector<CanFrame> encode_cycle(
  const MotionCommand & cmd, double front, double rear, SteerSource src, uint8_t counter,
  const HostRequest & req, const Config & cfg, const std::optional<LampCommand> & lamps,
  std::optional<uint64_t> unix_ms)
{
  std::vector<CanFrame> out;
  out.reserve(5);
  out.push_back(encode_motion(cmd.twist, cmd.model, cfg));
  out.push_back(encode_steer(cmd.gear, front, rear, src, cfg.accel_limit));
  out.push_back(encode_heartbeat(counter, req));
  if (lamps) {out.push_back(encode_lamps(*lamps));}
  if (unix_ms) {out.push_back(encode_host_time(*unix_ms));}
  return out;
}

bool decode(const CanFrame & f, const Config & cfg, ChassisState & s, Clock::time_point stamp)
{
  if (f.dlc < 8) {
    ++s.frames_unknown;
    return false;
  }
  const auto & d = f.data;
  switch (f.id) {
    case kMotionFb: {
        const double vx = be_i16(d[0], d[1]) / 1000.0;
        const double vy_ref = be_i16(d[2], d[3]) / 1000.0;
        const double wz = be_i16(d[4], d[5]) / 1000.0;
        // rigid body: v(base_link) = v(ref) + wz x (base_link - ref)
        s.motion.twist = {vx, vy_ref - wz * cfg.reference_x, wz};
        s.motion.model_raw = d[6];
        s.motion.control_raw = d[7];
        s.motion.source = source_from_raw(d[7]);
        s.motion_stamp = stamp;
        break;
      }
    case kSteerFb:
      s.steer.gear_raw = d[0];
      s.steer.gear = d[0] <= 4 ? static_cast<Gear>(d[0]) : Gear::kDisabled;
      s.steer.front_rad = be_i16(d[1], d[2]) / 1000.0;
      s.steer.rear_rad = be_i16(d[3], d[4]) / 1000.0;
      s.steer_stamp = stamp;
      break;
    case kLimitsFb:
      s.limits.max_vx_fwd = be_u16(d[0], d[1]) * 0.01;
      s.limits.max_vx_rev = be_u16(d[2], d[3]) * 0.01;
      s.limits.max_steer_rad = be_u16(d[4], d[5]) * 0.001;
      s.limits.reason_bits = d[6];
      s.limits.speed_clipped = d[7] & 0x01;
      s.limits.steer_clipped = d[7] & 0x02;
      s.limits.accel_clipped = d[7] & 0x04;
      s.limits_stamp = stamp;
      break;
    case kImuAccelFb:
      for (std::size_t i = 0; i < 3; ++i) {s.imu.accel[i] = be_i16(d[2 * i], d[2 * i + 1]) * 0.002;}
      s.imu.accel_counter = d[6];
      s.imu_accel_stamp = stamp;
      break;
    case kAttitudeFb:
      s.imu.roll = be_i16(d[0], d[1]) * 0.0001;
      s.imu.pitch = be_i16(d[2], d[3]) * 0.0001;
      s.attitude_stamp = stamp;
      break;
    case kImuRateFb:
      for (std::size_t i = 0; i < 3; ++i) {s.imu.rate[i] = be_i16(d[2 * i], d[2 * i + 1]) * 0.001;}
      s.imu.rate_counter = d[6];
      s.imu.status = d[7];
      s.imu_rate_stamp = stamp;
      break;
    case kProtectionFb:
      s.safety.ultrasonic_trigger_raw = d[0];
      s.safety.bumper_raw = d[1];
      s.safety.estop_raw = d[2];
      s.safety.fault_code = d[3];
      for (std::size_t i = 0; i < 4; ++i) {
        s.safety.ultrasonic_raw[i] = d[4 + i];
        s.safety.ultrasonic_m[i] = d[4 + i] == 255 ?
          std::numeric_limits<double>::infinity() : d[4 + i] * 0.01;
      }
      s.safety_stamp = stamp;
      break;
    case kWheelSpeedFb:   // order LF, RF, LR, RR = SDK order
      for (std::size_t w = 0; w < 4; ++w) {
        s.wheel_speed_raw[w] = be_i16(d[2 * w], d[2 * w + 1]);
        s.wheel_speed[w] = s.wheel_speed_raw[w] * 0.01;
      }
      s.wheel_speed_stamp = stamp;
      break;
    case kMotorFaultFb:
      s.motor_fault_high = (uint32_t{d[0]} << 24) | (uint32_t{d[1]} << 16) |
        (uint32_t{d[2]} << 8) | d[3];
      s.motor_fault_low = (uint32_t{d[4]} << 24) | (uint32_t{d[5]} << 16) |
        (uint32_t{d[6]} << 8) | d[7];
      s.motor_fault_stamp = stamp;
      break;
    case kBatteryFb: {
        const uint16_t soc = be_u16(d[0], d[1]);
        const uint16_t temp = be_u16(d[2], d[3]);
        const uint16_t chg = be_u16(d[4], d[5]);
        const uint16_t volt = be_u16(d[6], d[7]);
        s.battery.raw = {soc, temp, chg, volt};
        s.battery.soc_percent = soc * 0.01;
        s.battery.temperature_c = static_cast<int16_t>(temp) * 0.1;
        s.battery.charging_percent = chg;
        s.battery.voltage_v = volt * 0.1;
        s.battery_stamp = stamp;
        break;
      }
    case kChassisStateFb: {
        if (cfg.check_checksum && checksum(d) != d[7]) {
          ++s.status.checksum_errors;
          ++s.frames_unknown;
          return false;
        }
        if (s.status_stamp && d[6] != static_cast<uint8_t>(s.status.alive_counter + 1)) {
          ++s.status.counter_errors;
        }
        s.status.operating_mode = d[0];
        s.status.estop_source_bits = d[1];
        s.status.fault_level = d[2];
        s.status.rc_online = d[3] & 0x01;
        s.status.rc_enabled = d[3] & 0x02;
        s.status.four_wd = d[3] & 0x04;
        s.status.park_brake = d[3] & 0x08;
        s.status.rc_link = static_cast<uint8_t>((d[3] >> 4) & 0x03);
        s.status.cmd_clipped = d[3] & 0x40;
        s.status.active_speed_limit = be_u16(d[4], d[5]) * 0.01;
        s.status.alive_counter = d[6];
        s.status_stamp = stamp;
        break;
      }
    case kVersionFb:
      s.version = {d[0], d[1], d[2], d[3], d[4], d[5]};
      s.version_stamp = stamp;
      break;
    case kBrakeSteerFb:
      s.brake.brake_pressure_mpa = be_u16(d[0], d[1]) * 0.01;
      s.brake.epb_state = d[2];
      s.brake.eps_front = d[3];
      s.brake.eps_rear = d[4];
      s.brake.brake_state = d[5];
      s.brake_stamp = stamp;
      break;
    case kMotorTempFb:
      for (std::size_t w = 0; w < 4; ++w) {
        s.temperature.motor[w] = d[w] - 40.0;
        s.temperature.controller[w] = d[4 + w] - 40.0;
      }
      s.temperature_stamp = stamp;
      break;
    default:
      ++s.frames_unknown;
      return false;
  }
  ++s.frames_received;
  return true;
}

std::string describe(const CanFrame & f)
{
  char hex[3 * 8 + 1] = {0};
  for (std::size_t i = 0; i < f.dlc && i < 8; ++i) {
    std::snprintf(hex + 3 * i, 4, "%02X ", f.data[i]);
  }
  const char * name = "unknown";
  switch (f.id) {
    case kMotionCmd: name = "MotionCmd"; break;
    case kSteerCmd: name = "SteerCmd"; break;
    case kLampCmd: name = "LampCmd"; break;
    case kHostTime: name = "HostTime"; break;
    case kHeartbeat: name = "HostHeartbeat"; break;
    case kLimitsFb: name = "LimitsFb"; break;
    case kImuAccelFb: name = "ImuAccelFb"; break;
    case kAttitudeFb: name = "AttitudeFb"; break;
    case kMotionFb: name = "MotionFb"; break;
    case kSteerFb: name = "SteerFb"; break;
    case kProtectionFb: name = "ProtectionFb"; break;
    case kWheelSpeedFb: name = "WheelSpeedFb"; break;
    case kMotorFaultFb: name = "MotorFault"; break;
    case kBatteryFb: name = "BatteryFb"; break;
    case kChassisStateFb: name = "ChassisState"; break;
    case kImuRateFb: name = "ImuRateFb"; break;
    case kVersionFb: name = "Version"; break;
    case kBrakeSteerFb: name = "B_BrakeSteerFb"; break;
    case kMotorTempFb: name = "B_MotorTempFb"; break;
    default: break;
  }
  char out[96];
  std::snprintf(out, sizeof(out), "0x%03X %-14s [%u] %s", f.id, name, f.dlc, hex);
  return out;
}

}  // namespace lci
}  // namespace luchs_sdk
