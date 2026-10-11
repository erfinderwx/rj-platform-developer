// Copyright 2026 TerraAgent Robotics. Chassis: RJ Tech Limited.
// SPDX-License-Identifier: Apache-2.0

#include "luchs_sdk/sim_bus.hpp"

#include <algorithm>
#include <cmath>
#include <thread>

namespace luchs_sdk
{
using namespace legacy_a;

SimBus::SimBus(Geometry g, legacy_a::Config cfg, double tau)
: g_(g), cfg_(cfg), tau_(std::max(tau, 0.01)), model_(default_model(g.build)) {}

void SimBus::press_estop(bool pressed)
{
  std::lock_guard<std::mutex> lk(mtx_);
  estop_ = pressed;
}

bool SimBus::write(const CanFrame & f)
{
  std::lock_guard<std::mutex> lk(mtx_);
  const auto & d = f.data;
  if (f.id == kMotionCmd) {
    target_ = {be_i16(d[0], d[1]) / 1000.0, be_i16(d[2], d[3]) / 1000.0,
      be_i16(d[4], d[5]) / 1000.0};
    if (d[6] >= 1 && d[6] <= 4) {model_ = static_cast<MotionModel>(d[6]);}
  } else if (f.id == kSteerCmd) {
    gear_ = d[0];
    front_ = be_i16(d[1], d[2]) / 1000.0;
    rear_ = be_i16(d[3], d[4]) / 1000.0;
  } else if (f.id == cfg_.lamp_cmd_id()) {
    lamps_.turn = static_cast<TurnSignal>(std::min<uint8_t>(d[0], 2));
    lamps_.reverse = d[1] != 0;
    lamps_.fault = d[2] != 0;
  }
  return true;
}

void SimBus::step(Clock::time_point now)
{
  const double dt = last_ ? std::chrono::duration<double>(now - *last_).count() : 0.0;
  last_ = now;
  // Park, a disabled gear or the e-stop holds the vehicle.
  const bool hold = estop_ || gear_ == 1 || gear_ == 0;
  const Twist2D tgt = hold ? Twist2D{} : target_;
  const double k = 1.0 - std::exp(-dt / tau_);
  actual_.vx += (tgt.vx - actual_.vx) * k;
  actual_.vy += (tgt.vy - actual_.vy) * k;
  actual_.wz += (tgt.wz - actual_.wz) * k;
  soc_ = std::max(0.0, soc_ - dt * 1e-4);
}

std::optional<CanFrame> SimBus::read(int timeout_ms)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    const auto now = Clock::now();
    if (rx_.empty() && now >= next_emit_) {
      step(now);
      next_emit_ = now + std::chrono::milliseconds(20);
      const SteerSolution sol = solve(actual_, model_, g_);
      auto put = [this](uint32_t id, std::array<uint8_t, 8> data) {
          rx_.push_back(CanFrame{id, 8, data});
        };
      std::array<uint8_t, 8> d{};
      put_i16(d, 0, to_raw16(actual_.vx));
      put_i16(d, 2, to_raw16(actual_.vy));
      put_i16(d, 4, to_raw16(actual_.wz));
      d[6] = static_cast<uint8_t>(model_);
      d[7] = 0x22;
      put(kMotionFb, d);
      d.fill(0);
      d[0] = gear_;
      put_i16(d, 1, to_raw16(sol.front_rad));
      put_i16(d, 3, to_raw16(sol.rear_rad));
      put(kSteerFb, d);
      d.fill(0);
      // 0x5B7 in the configured feedback order
      const bool lflr = cfg_.wheel_steer_fb_order == WheelOrder::kLfLrRfRr;
      const std::array<std::size_t, 4> slot = lflr ? std::array<std::size_t, 4>{0, 2, 1, 3} :
        std::array<std::size_t, 4>{0, 1, 2, 3};
      for (std::size_t w = 0; w < 4; ++w) {put_i16(d, 2 * slot[w], to_raw16(sol.wheel_rad[w]));}
      put(kWheelSteerFb, d);
      d.fill(0);
      const bool slf = cfg_.wheel_speed_fb_order == WheelOrder::kLfLrRfRr;
      const std::array<std::size_t, 4> s2 = slf ? std::array<std::size_t, 4>{0, 2, 1, 3} :
        std::array<std::size_t, 4>{0, 1, 2, 3};
      for (std::size_t w = 0; w < 4; ++w) {
        const double rad_s = sol.wheel_speed[w] / g_.wheel_radius;
        put_i16(d, 2 * s2[w], to_raw16(rad_s, 1.0 / cfg_.wheel_speed_scale));
      }
      put(kWheelSpeedFb, d);
      d = {0, 0, static_cast<uint8_t>(estop_ ? 1 : 0), static_cast<uint8_t>(estop_ ? 0x02 : 0x00),
        150, 200, 255, 255};
      put(kProtectionFb, d);
      const auto soc = static_cast<uint16_t>(soc_ / cfg_.battery_soc_scale);
      const auto temp = static_cast<uint16_t>(static_cast<int16_t>(25.0 / cfg_.battery_temp_scale));
      const auto volt = static_cast<uint16_t>(51.2 / cfg_.battery_voltage_scale);
      d = {static_cast<uint8_t>(soc >> 8), static_cast<uint8_t>(soc), static_cast<uint8_t>(temp >> 8),
        static_cast<uint8_t>(temp), 0, 0, static_cast<uint8_t>(volt >> 8), static_cast<uint8_t>(volt)};
      put(kBatteryFb, d);
      d = {static_cast<uint8_t>(lamps_.turn), lamps_.reverse, lamps_.fault, 0x10, 0, 0, 0, 0};
      put(kLampFb, d);
    }
    if (!rx_.empty()) {
      CanFrame f = rx_.front();
      rx_.pop_front();
      return f;
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(std::clamp(timeout_ms, 0, 5)));
  return std::nullopt;
}

}  // namespace luchs_sdk
