// Copyright 2026 TerraAgent Robotics. Chassis: RJ Tech Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Simulators for the Luchs B interfaces (LCI and the VCU upper interface), and the factory.

#include <algorithm>
#include <cmath>
#include <thread>

#include "luchs_sdk/lci_protocol.hpp"
#include "luchs_sdk/sim_bus.hpp"

namespace luchs_sdk
{
using legacy_a::be_i16;
using legacy_a::be_u16;
using legacy_a::put_i16;
using legacy_a::to_raw16;

namespace
{
double secs(Clock::duration d) {return std::chrono::duration<double>(d).count();}
void put_u16(std::array<uint8_t, 8> & d, std::size_t at, double v, double scale)
{
  const auto u = static_cast<uint16_t>(std::clamp(std::round(v * scale), 0.0, 65535.0));
  d[at] = static_cast<uint8_t>(u >> 8);
  d[at + 1] = static_cast<uint8_t>(u & 0xFF);
}
constexpr double kMaxFwd = 40.0 / 3.6;     // Luchs B: 40 km/h
constexpr double kMaxRev = 10.0 / 3.6;     // reverse limited to 10 km/h
constexpr double kHeartbeatTimeout = 0.2;  // s
constexpr double kSteerRate = 1.0;         // rad/s road-wheel slew
}  // namespace

// ============================== LCI ==========================================================

SimBusLci::SimBusLci(Geometry g, double tau)
: g_(g), tau_(std::max(tau, 0.01)), model_(default_model(g.build)) {}

void SimBusLci::press_estop(bool p) {std::lock_guard<std::mutex> lk(mtx_); estop_button_ = p;}
void SimBusLci::set_remote_control(bool rc) {std::lock_guard<std::mutex> lk(mtx_); rc_ = rc;}
void SimBusLci::maintenance_reset() {std::lock_guard<std::mutex> lk(mtx_); estop_latched_ = false;}
void SimBusLci::set_terrain_speed_limit(double v)
{
  std::lock_guard<std::mutex> lk(mtx_);
  terrain_limit_ = std::max(0.0, v);
}

bool SimBusLci::write(const CanFrame & f)
{
  std::lock_guard<std::mutex> lk(mtx_);
  const auto & d = f.data;
  const auto now = Clock::now();
  switch (f.id) {
    case lci::kMotionCmd:
      target_ = {be_i16(d[0], d[1]) / 1000.0, be_i16(d[2], d[3]) / 1000.0, be_i16(d[4], d[5]) / 1000.0};
      if (model_supported(g_.build, static_cast<MotionModel>(d[6]))) {
        model_ = static_cast<MotionModel>(d[6]);
      }
      break;
    case lci::kSteerCmd:
      gear_ = d[0];
      cmd_front_ = be_i16(d[1], d[2]) / 1000.0;
      cmd_rear_ = be_i16(d[3], d[4]) / 1000.0;
      steer_source_ = d[5];
      break;
    case lci::kLampCmd:
      lamps_.turn = static_cast<TurnSignal>(std::min<uint8_t>(d[0], 3));
      lamps_.headlight = d[3];
      break;
    case lci::kHeartbeat:
      if (lci::checksum(d) != d[7]) {break;}     // a corrupt heartbeat does not count
      if (!last_counter_ || *last_counter_ != d[0]) {counter_changed_ = now;}
      last_counter_ = d[0];
      last_hb_ = now;
      req_.estop = d[1] & 0x01;
      req_.park = d[1] & 0x02;
      req_.payload_power_1 = d[1] & 0x08;
      req_.payload_power_2 = d[1] & 0x10;
      req_.speed_limit = be_u16(d[2], d[3]) * 0.01;
      if (req_.estop) {estop_latched_ = true;}
      break;
    default: break;
  }
  return true;
}

void SimBusLci::step(Clock::time_point now)
{
  const double dt = last_ ? std::min(secs(now - *last_), 0.1) : 0.0;
  last_ = now;
  hb_timeout_ = !last_hb_ || secs(now - *last_hb_) > kHeartbeatTimeout ||
    secs(now - counter_changed_) > kHeartbeatTimeout;

  // live limits
  double fwd = kMaxFwd, rev = kMaxRev;
  if (req_.speed_limit > 0.0) {fwd = std::min(fwd, req_.speed_limit); rev = std::min(rev, req_.speed_limit);}
  if (terrain_limit_ > 0.0) {fwd = std::min(fwd, terrain_limit_); rev = std::min(rev, terrain_limit_);}
  const double steer_lim = g_.max_steer_rad;

  const bool host = !rc_;
  const bool hold = estop_button_ || estop_latched_ || (host && (hb_timeout_ || req_.park)) ||
    gear_ <= 1 || gear_ == 3;
  double tvx = host && !hold ? target_.vx : 0.0;
  if (gear_ == 2 && tvx > 0.0) {tvx = -tvx;}    // R: reverse
  speed_clipped_ = tvx > fwd + 1e-6 || tvx < -rev - 1e-6;
  tvx = std::clamp(tvx, -rev, fwd);
  const double k = 1.0 - std::exp(-dt / tau_);
  vx_ += (tvx - vx_) * k;
  if (hold && std::abs(vx_) < 0.02) {vx_ = 0.0;}

  // steering: angle mode, or from the yaw rate at the reference point
  double tf = front_, tr = rear_;
  if (host && !hold && steer_source_ == 1) {
    tf = cmd_front_;
    tr = model_ == MotionModel::kDualAckermann ? cmd_rear_ : 0.0;
  } else if (host && !hold && std::abs(target_.vx) > 0.05) {
    const double L = g_.wheelbase;
    if (model_ == MotionModel::kDualAckermann) {
      tf = std::atan(L * target_.wz / (2.0 * target_.vx));
      tr = -tf;
    } else {
      tf = std::atan(L * target_.wz / target_.vx);
      tr = 0.0;
    }
  }
  steer_clipped_ = std::abs(tf) > steer_lim + 1e-6 || std::abs(tr) > steer_lim + 1e-6;
  tf = std::clamp(tf, -steer_lim, steer_lim);
  tr = std::clamp(tr, -steer_lim, steer_lim);
  front_ += std::clamp(tf - front_, -kSteerRate * dt, kSteerRate * dt);
  rear_ += std::clamp(tr - rear_, -kSteerRate * dt, kSteerRate * dt);
  wz_ = vx_ * (std::tan(front_) - std::tan(rear_)) / g_.wheelbase;
  soc_ = std::max(0.0, soc_ - dt * 1e-4);
}

void SimBusLci::emit(Clock::time_point now)
{
  (void)now;
  ++ticks_;
  auto put = [this](uint32_t id, const std::array<uint8_t, 8> & data) {
      rx_.push_back(CanFrame{id, 8, data});
    };
  const double L = g_.wheelbase;
  std::array<uint8_t, 8> d{};
  // 0x5B5 motion at the rear-axle centre: vy = wz * (x_ICR offset); with front angle f and
  // rear angle r the rear-axle centre moves sideways by vx * tan(r).
  put_i16(d, 0, to_raw16(vx_));
  put_i16(d, 2, to_raw16(vx_ * std::tan(rear_)));
  put_i16(d, 4, to_raw16(wz_));
  d[6] = static_cast<uint8_t>(model_);
  d[7] = rc_ ? 0x11 : 0x22;
  put(lci::kMotionFb, d);
  d.fill(0);
  d[0] = gear_;
  put_i16(d, 1, to_raw16(front_));
  put_i16(d, 3, to_raw16(rear_));
  put(lci::kSteerFb, d);
  // 0x5B0 live limits
  d.fill(0);
  double fwd = kMaxFwd, rev = kMaxRev;
  uint8_t reason = 0;
  if (req_.speed_limit > 0.0 && req_.speed_limit < fwd) {fwd = req_.speed_limit; reason |= 0x10;}
  if (req_.speed_limit > 0.0) {rev = std::min(rev, req_.speed_limit);}
  if (terrain_limit_ > 0.0 && terrain_limit_ < fwd) {fwd = terrain_limit_; reason |= 0x01;}
  if (terrain_limit_ > 0.0) {rev = std::min(rev, terrain_limit_);}
  put_u16(d, 0, fwd, 100.0);
  put_u16(d, 2, rev, 100.0);
  put_u16(d, 4, g_.max_steer_rad, 1000.0);
  d[6] = reason;
  d[7] = static_cast<uint8_t>((speed_clipped_ ? 1 : 0) | (steer_clipped_ ? 2 : 0));
  put(lci::kLimitsFb, d);
  // 0x5B9 wheel speeds (rad/s), LF RF LR RR
  d.fill(0);
  const auto pos = wheel_positions(g_);
  for (std::size_t w = 0; w < 4; ++w) {
    // ground speed of each wheel contact about the instantaneous centre
    const double vxw = vx_ - wz_ * pos[w][1];
    const double vyw = vx_ * std::tan(rear_) + wz_ * (pos[w][0] + L / 2.0);
    const double s = std::copysign(std::hypot(vxw, vyw), vx_);
    put_i16(d, 2 * w, to_raw16(s / g_.wheel_radius, 100.0));
  }
  put(lci::kWheelSpeedFb, d);
  // 0x5BD chassis state
  d.fill(0);
  d[0] = rc_ ? 1 : 2;
  d[1] = static_cast<uint8_t>((estop_button_ ? 0x01 : 0) | (estop_latched_ ? 0x08 : 0) |
    (!rc_ && hb_timeout_ && last_hb_ ? 0x80 : 0));
  d[2] = estop_latched_ || estop_button_ ? 3 : 0;
  const bool parked = std::abs(vx_) < 0.02 && (gear_ == 1 || req_.park || hb_timeout_ ||
    estop_latched_ || estop_button_);
  d[3] = static_cast<uint8_t>(0x01 | (rc_ ? 0x02 : 0) | 0x04 | (parked ? 0x08 : 0) |
    ((speed_clipped_ || steer_clipped_) ? 0x40 : 0));   // RC online, 4WD, RCLink good
  put_u16(d, 4, fwd, 100.0);
  d[6] = alive_++;
  d[7] = lci::checksum(d);
  put(lci::kChassisStateFb, d);
  // IMU 0x5BE / 0x5B1 (paired by counter) and attitude 0x5B2
  d.fill(0);
  put_i16(d, 4, to_raw16(wz_));
  d[6] = imu_counter_;
  put(lci::kImuRateFb, d);
  d.fill(0);
  put_i16(d, 2, to_raw16(vx_ * wz_, 500.0));
  put_i16(d, 4, to_raw16(9.81, 500.0));
  d[6] = imu_counter_++;
  put(lci::kImuAccelFb, d);
  d.fill(0);
  put(lci::kAttitudeFb, d);
  if (ticks_ % 2 == 0) {   // ~50 ms
    d = {0, 0, static_cast<uint8_t>(estop_button_ ? 1 : 0), static_cast<uint8_t>(estop_button_ ? 0x02 : 0),
      255, 255, 255, 255};
    put(lci::kProtectionFb, d);
    d.fill(0);
    put_u16(d, 0, std::abs(vx_) > 0.02 ? 0.0 : 1.2, 100.0);
    d[2] = parked ? 2 : 0;
    put(lci::kBrakeSteerFb, d);
  }
  if (ticks_ % 5 == 0) {   // 100 ms
    d.fill(0);
    put_u16(d, 0, soc_, 100.0);
    const auto temp = static_cast<uint16_t>(static_cast<int16_t>(28.0 * 10.0));
    d[2] = static_cast<uint8_t>(temp >> 8);
    d[3] = static_cast<uint8_t>(temp);
    put_u16(d, 6, 76.8, 10.0);
    put(lci::kBatteryFb, d);
    d.fill(0);
    put(lci::kMotorFaultFb, d);
  }
  if (ticks_ % 25 == 0) {  // 500 ms
    d = {75, 76, 74, 75, 70, 71, 70, 70};   // 35/36/34/35 and 30/31/30/30 deg C
    put(lci::kMotorTempFb, d);
  }
  if (ticks_ % 50 == 1) {  // 1 s
    d = {lci::kLciMajor, lci::kLciMinor, lci::model_id_for(g_.build), 0, 9, 0, 0, 0};
    put(lci::kVersionFb, d);
  }
}

std::optional<CanFrame> SimBusLci::read(int timeout_ms)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    const auto now = Clock::now();
    if (rx_.empty() && now >= next_emit_) {
      step(now);
      emit(now);
      next_emit_ = now + std::chrono::milliseconds(20);
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

// ============================== legacy B ======================================================

SimBusLegacyB::SimBusLegacyB(Geometry g, legacy_b::Config cfg)
: g_(g), cfg_(cfg) {}

double SimBusLegacyB::speed() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return gear_ == 2 ? -v_ : v_;
}

bool SimBusLegacyB::write(const CanFrame & f)
{
  if (f.id != cfg_.cmd_id) {return true;}
  std::lock_guard<std::mutex> lk(mtx_);
  const auto & d = f.data;
  throttle_ = d[0];
  brake_ = d[1];
  cap_kmh_ = d[3];
  remote_ = d[6] & 0x01;
  estop_ = d[6] & 0x04;
  park_ = d[6] & 0x08;
  const uint8_t g = (d[6] >> 4) & 0x0F;
  if (v_ < 0.05 || g == gear_) {gear_ = g;}   // the VCU ignores a direction change while moving
  last_cmd_ = Clock::now();
  return true;
}

std::optional<CanFrame> SimBusLegacyB::read(int timeout_ms)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    const auto now = Clock::now();
    if (rx_.empty() && now >= next_emit_) {
      const double dt = last_ ? std::min(secs(now - *last_), 0.1) : 0.0;
      last_ = now;
      const bool timeout = !last_cmd_ || secs(now - *last_cmd_) > 0.4;
      const bool hold = timeout || estop_ || park_ || !remote_;
      double a = -0.3 * v_;                                  // rolling resistance / drag
      if (!hold && (gear_ == 1 || gear_ == 2)) {a += 0.05 * throttle_;}
      a -= (hold ? 4.0 : 0.08 * brake_) * (v_ > 0.0 ? 1.0 : 0.0);
      v_ = std::max(0.0, v_ + a * dt);
      if (cap_kmh_ > 0.0) {v_ = std::min(v_, cap_kmh_ / 3.6);}
      next_emit_ = now + std::chrono::milliseconds(10);
      std::array<uint8_t, 8> d{};
      d[0] = 86;
      d[1] = 28 + 40;
      d[2] = 76;
      d[3] = 0;
      d[5] = static_cast<uint8_t>(((gear_ & 0x03) << 2) | (remote_ ? 0x10 : 0));
      d[6] = static_cast<uint8_t>(std::lround(v_ * 3.6));
      d[7] = hb_++;
      rx_.push_back(CanFrame{cfg_.fb_id, 8, d});
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

// ============================== factory =======================================================

std::unique_ptr<Transport> make_simulator(const ChassisOptions & o)
{
  switch (o.protocol_type) {
    case Protocol::kLegacyA: return std::make_unique<SimBus>(o.geometry, o.protocol);
    case Protocol::kLci: return std::make_unique<SimBusLci>(o.geometry);
    case Protocol::kLegacyB: return std::make_unique<SimBusLegacyB>(o.geometry, o.legacy_b);
  }
  return nullptr;
}

}  // namespace luchs_sdk
