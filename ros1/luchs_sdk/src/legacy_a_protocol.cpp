// Copyright 2026 TerraAgent Robotics. Chassis: RJ Tech Limited.
// SPDX-License-Identifier: Apache-2.0

#include "luchs_sdk/legacy_a_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace luchs_sdk
{

std::string fault_code_text(uint8_t code)
{
  switch (code) {
    case 0x00: return "no fault";
    case 0x01: return "ultrasonic stop";
    case 0x02: return "emergency stop pressed";
    case 0x03: return "parked";
    case 0x04: return "charging";
    case 0x05: return "front-left drive motor fault";
    case 0x06: return "front-right drive motor fault";
    case 0x07: return "rear-left drive motor fault";
    case 0x08: return "rear-right drive motor fault";
    case 0x09: return "front-left steering motor fault";
    case 0x0A: return "front-right steering motor fault";
    case 0x0B: return "rear-left steering motor fault";
    case 0x0C: return "rear-right steering motor fault";
    case 0x0D: return "remote-control link lost";
    case 0x0E: return "watchdog";
    default: {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "unknown fault 0x%02X", code);
        return buf;
      }
  }
}

namespace legacy_a
{
namespace
{
// SDK order is LF, RF, LR, RR. Map SDK index -> wire slot for each wire order.
constexpr std::array<std::size_t, 4> kSlotLfRfLrRr{0, 1, 2, 3};
constexpr std::array<std::size_t, 4> kSlotLfLrRfRr{0, 2, 1, 3};
const std::array<std::size_t, 4> & slots(WheelOrder o)
{
  return o == WheelOrder::kLfRfLrRr ? kSlotLfRfLrRr : kSlotLfLrRfRr;
}
CanFrame frame(uint32_t id)
{
  CanFrame f;
  f.id = id;
  f.dlc = 8;
  f.data.fill(0);
  return f;
}
ControlSource source_from_raw(uint8_t raw)
{
  // The document writes "11 remote control, 22 autonomous" without a radix. Accept both.
  if (raw == 0x11 || raw == 11) {return ControlSource::kRemoteControl;}
  if (raw == 0x22 || raw == 22) {return ControlSource::kHost;}
  return ControlSource::kUnknown;
}
}  // namespace

int16_t to_raw16(double value, double scale)
{
  if (!std::isfinite(value)) {return 0;}
  const double r = std::round(value * scale);
  const double lo = std::numeric_limits<int16_t>::min();
  const double hi = std::numeric_limits<int16_t>::max();
  return static_cast<int16_t>(std::clamp(r, lo, hi));
}

CanFrame encode_motion(const Twist2D & t, MotionModel model, uint8_t control_mode)
{
  CanFrame f = frame(kMotionCmd);
  put_i16(f.data, 0, to_raw16(t.vx));
  put_i16(f.data, 2, to_raw16(t.vy));
  put_i16(f.data, 4, to_raw16(t.wz));
  f.data[6] = static_cast<uint8_t>(model);
  f.data[7] = control_mode;
  return f;
}

CanFrame encode_steer(Gear gear, double front_rad, double rear_rad)
{
  CanFrame f = frame(kSteerCmd);
  f.data[0] = static_cast<uint8_t>(gear);
  put_i16(f.data, 1, to_raw16(front_rad));
  put_i16(f.data, 3, to_raw16(rear_rad));
  return f;
}

CanFrame encode_wheel_steer(const PerWheel & a, const Config & cfg)
{
  CanFrame f = frame(cfg.wheel_steer_cmd_id);
  const auto & s = slots(cfg.wheel_steer_cmd_order);
  for (std::size_t w = 0; w < 4; ++w) {
    put_i16(f.data, 2 * s[w], to_raw16(a[w]));
  }
  return f;
}

CanFrame encode_lamps(const LampCommand & l, const Config & cfg)
{
  CanFrame f = frame(cfg.lamp_cmd_id());
  f.data[0] = static_cast<uint8_t>(l.turn);
  f.data[1] = l.reverse ? 1 : 0;
  f.data[2] = l.fault ? 1 : 0;
  return f;
}

CanFrame encode_host_time(uint64_t ms)
{
  CanFrame f = frame(kHostTimeCmd);
  for (std::size_t i = 0; i < 8; ++i) {
    f.data[i] = static_cast<uint8_t>((ms >> (8 * (7 - i))) & 0xFF);
  }
  return f;
}

std::vector<CanFrame> encode_cycle(
  const MotionCommand & cmd, const PerWheel & wheel_rad, double front_rad, double rear_rad,
  const LampCommand & lamps, const Config & cfg, std::optional<uint64_t> unix_ms)
{
  std::vector<CanFrame> out;
  out.reserve(5);
  out.push_back(encode_motion(cmd.twist, cmd.model));
  out.push_back(encode_steer(cmd.gear, front_rad, rear_rad));
  out.push_back(encode_wheel_steer(wheel_rad, cfg));
  out.push_back(encode_lamps(lamps, cfg));
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
    case kMotionFb:
      s.motion.twist.vx = be_i16(d[0], d[1]) / 1000.0;
      s.motion.twist.vy = be_i16(d[2], d[3]) / 1000.0;
      s.motion.twist.wz = be_i16(d[4], d[5]) / 1000.0;
      s.motion.model_raw = d[6];
      s.motion.control_raw = d[7];
      s.motion.source = source_from_raw(d[7]);
      s.motion_stamp = stamp;
      break;
    case kSteerFb:
      s.steer.gear_raw = d[0];
      s.steer.gear = d[0] <= 4 ? static_cast<Gear>(d[0]) : Gear::kDisabled;
      s.steer.front_rad = be_i16(d[1], d[2]) / 1000.0;
      s.steer.rear_rad = be_i16(d[3], d[4]) / 1000.0;
      s.steer_stamp = stamp;
      break;
    case kWheelSteerFb: {
        const auto & sl = slots(cfg.wheel_steer_fb_order);
        for (std::size_t w = 0; w < 4; ++w) {
          s.wheel_steer_rad[w] = be_i16(d[2 * sl[w]], d[2 * sl[w] + 1]) / 1000.0;
        }
        s.wheel_steer_stamp = stamp;
        break;
      }
    case kProtectionFb:
      s.safety.ultrasonic_trigger_raw = d[0];
      s.safety.bumper_raw = d[1];
      s.safety.estop_raw = d[2];
      s.safety.fault_code = d[3];
      for (std::size_t i = 0; i < 4; ++i) {
        s.safety.ultrasonic_raw[i] = d[4 + i];
        s.safety.ultrasonic_m[i] = d[4 + i] * cfg.ultrasonic_scale_m;
      }
      s.safety_stamp = stamp;
      break;
    case kWheelSpeedFb: {
        const auto & sl = slots(cfg.wheel_speed_fb_order);
        for (std::size_t w = 0; w < 4; ++w) {
          s.wheel_speed_raw[w] = be_i16(d[2 * sl[w]], d[2 * sl[w] + 1]);
          s.wheel_speed[w] = s.wheel_speed_raw[w] * cfg.wheel_speed_scale;
        }
        s.wheel_speed_stamp = stamp;
        break;
      }
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
        s.battery.soc_percent = soc * cfg.battery_soc_scale;
        s.battery.temperature_c = static_cast<int16_t>(temp) * cfg.battery_temp_scale;
        s.battery.charging_percent = chg * cfg.charging_scale;
        s.battery.voltage_v = volt * cfg.battery_voltage_scale;
        s.battery_stamp = stamp;
        break;
      }
    case kLampFb:
      s.lamps.turn_raw = d[0];
      s.lamps.reverse_raw = d[1];
      s.lamps.fault_raw = d[2];
      s.lamps.light_rain_raw = d[3];
      s.lamps_stamp = stamp;
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
    case kId5A2: name = "Cmd_0x5A2"; break;
    case kId5A3: name = "Cmd_0x5A3"; break;
    case kHostTimeCmd: name = "HostTime"; break;
    case kMotionFb: name = "MotionFb"; break;
    case kSteerFb: name = "SteerFb"; break;
    case kWheelSteerFb: name = "WheelSteerFb"; break;
    case kProtectionFb: name = "ProtectionFb"; break;
    case kWheelSpeedFb: name = "WheelSpeedFb"; break;
    case kMotorFaultFb: name = "MotorFaultFb"; break;
    case kBatteryFb: name = "BatteryFb"; break;
    case kLampFb: name = "LampFb"; break;
    default: break;
  }
  char out[96];
  std::snprintf(out, sizeof(out), "0x%03X %-13s [%u] %s", f.id, name, f.dlc, hex);
  return out;
}

}  // namespace legacy_a
}  // namespace luchs_sdk
