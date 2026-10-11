// Copyright 2026 TerraAgent Robotics. Chassis: RJ Tech Limited.
// SPDX-License-Identifier: Apache-2.0

#include "luchs_sdk/chassis.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace luchs_sdk
{
namespace
{
double seconds(Clock::duration d) {return std::chrono::duration<double>(d).count();}
bool is_zero(const Twist2D & t)
{
  return std::abs(t.vx) < 1e-4 && std::abs(t.vy) < 1e-4 && std::abs(t.wz) < 1e-4;
}
constexpr double kLiveLimitMaxAge = 0.2;   // s
}  // namespace

ChassisOptions ChassisOptions::for_build(Build b, Protocol p)
{
  ChassisOptions o;
  o.geometry = Geometry::for_build(b);
  o.protocol_type = p;
  if (p == Protocol::kLci) {o.tx_rate_hz = 50.0;}          // LCI: commands + heartbeat every 20 ms
  if (p == Protocol::kLegacyB) {o.tx_rate_hz = 100.0;}     // 0x701 cycle 10 ms
  if (b == Build::kLuchsB) {
    o.max_accel_x = 1.0;
    o.max_accel_y = 0.0;
    o.max_accel_wz = 1.0;
  }
  return o;
}

ChassisOptions ChassisOptions::for_build(Build b)
{
  return for_build(b, b == Build::kLuchsB ? Protocol::kLci : Protocol::kLegacyA);
}

uint32_t motion_feedback_id(const ChassisOptions & o)
{
  switch (o.protocol_type) {
    case Protocol::kLegacyA: return legacy_a::kMotionFb;
    case Protocol::kLci: return lci::kMotionFb;
    case Protocol::kLegacyB: return o.legacy_b.fb_id;
  }
  return 0;
}

Chassis::Chassis(ChassisOptions options)
: Chassis(std::move(options), nullptr) {}

Chassis::Chassis(ChassisOptions options, std::unique_ptr<Transport> transport)
: opt_(std::move(options)), can_(std::move(transport)),
  slew_(opt_.max_accel_x, opt_.max_accel_y, opt_.max_accel_wz)
{
  opt_.tx_rate_hz = std::clamp(opt_.tx_rate_hz, 1.0, 100.0);
  opt_.lci.reference_x = -opt_.geometry.wheelbase / 2.0;
  const MotionModel m = opt_.model.value_or(default_model(opt_.geometry.build));
  cmd_.model = model_supported(opt_.geometry.build, m) ? m : default_model(opt_.geometry.build);
  cmd_.gear = Gear::kDrive;
  status_.model = cmd_.model;
  status_.protocol = opt_.protocol_type;
}

Chassis::~Chassis() {stop();}

bool Chassis::start()
{
  if (running_) {return true;}
  if (!can_) {
    auto sock = std::make_unique<SocketCan>();
    if (!sock->open(opt_.interface)) {
      std::lock_guard<std::mutex> lk(mtx_);
      error_ = sock->last_error();
      return false;
    }
    can_ = std::move(sock);
  }
  {
    std::lock_guard<std::mutex> lk(mtx_);
    status_.can_open = can_->is_open();
  }
  running_ = true;
  rx_ = std::thread(&Chassis::rx_loop, this);
  tx_ = std::thread(&Chassis::tx_loop, this);
  return true;
}

void Chassis::stop()
{
  if (!running_.exchange(false)) {return;}
  if (tx_.joinable()) {tx_.join();}
  if (rx_.joinable()) {rx_.join();}
  // Final stop: zero motion, gear P (and a park request where the protocol has one), three
  // times to survive a dropped frame.
  if (!(can_ && can_->is_open())) {return;}
  std::vector<CanFrame> frames;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    MotionCommand stop_cmd;
    stop_cmd.model = cmd_.model;
    stop_cmd.gear = Gear::kPark;
    HostRequest req = host_;
    req.park = true;
    for (int i = 0; i < 3; ++i) {
      std::vector<CanFrame> one;
      switch (opt_.protocol_type) {
        case Protocol::kLegacyA:
          one = legacy_a::encode_cycle(stop_cmd, PerWheel{}, 0.0, 0.0, LampCommand{}, opt_.protocol);
          break;
        case Protocol::kLci:
          one = lci::encode_cycle(stop_cmd, 0.0, 0.0, lci::SteerSource::kYawRate, ++counter_, req,
              opt_.lci);
          break;
        case Protocol::kLegacyB:
          one = {legacy_b::encode_command(legacy_b::Pedals{0.0, opt_.legacy_b.hold_brake}, 0.0,
              0.0, Gear::kPark, req, lamps_, ++counter_, opt_.legacy_b)};
          break;
      }
      frames.insert(frames.end(), one.begin(), one.end());
    }
  }
  for (const auto & f : frames) {can_->write(f);}
}

void Chassis::set_twist(const Twist2D & twist, Clock::time_point stamp)
{
  std::lock_guard<std::mutex> lk(mtx_);
  cmd_.twist = twist;
  cmd_.front_steer_rad.reset();
  cmd_.rear_steer_rad.reset();
  cmd_.wheel_steer_rad.reset();
  cmd_.gear = Gear::kDrive;
  cmd_explicit_ = false;
  cmd_stamp_ = stamp;
}

void Chassis::set_command(const MotionCommand & command, Clock::time_point stamp)
{
  std::lock_guard<std::mutex> lk(mtx_);
  const MotionModel keep = cmd_.model;
  cmd_ = command;
  if (!model_supported(opt_.geometry.build, cmd_.model)) {cmd_.model = keep;}
  cmd_explicit_ = true;
  cmd_stamp_ = stamp;
}

void Chassis::set_model(MotionModel model)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (model_supported(opt_.geometry.build, model)) {cmd_.model = model;}
}

void Chassis::set_lamps(const LampCommand & lamps)
{
  std::lock_guard<std::mutex> lk(mtx_);
  lamps_ = lamps;
  last_lamps_tx_.reset();   // send the change in the next cycle
}

void Chassis::halt()
{
  std::lock_guard<std::mutex> lk(mtx_);
  cmd_.twist = Twist2D{};
  cmd_stamp_.reset();
  slew_.reset();
  speed_ctl_.reset();
}

void Chassis::set_host_request(const HostRequest & req)
{
  std::lock_guard<std::mutex> lk(mtx_);
  host_ = req;
}

void Chassis::request_estop(bool on)
{
  std::lock_guard<std::mutex> lk(mtx_);
  host_.estop = on;
}

void Chassis::request_park(bool on)
{
  std::lock_guard<std::mutex> lk(mtx_);
  host_.park = on;
}

void Chassis::set_payload_power(int output, bool on)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (output == 1) {host_.payload_power_1 = on;}
  if (output == 2) {host_.payload_power_2 = on;}
}

void Chassis::set_speed_limit(double mps)
{
  std::lock_guard<std::mutex> lk(mtx_);
  host_.speed_limit = std::isfinite(mps) ? std::max(0.0, mps) : 0.0;
}

ChassisState Chassis::state() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return state_;
}

Chassis::Status Chassis::status() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return status_;
}

std::string Chassis::last_error() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return error_;
}

void Chassis::on_feedback(FeedbackCallback cb)
{
  std::lock_guard<std::mutex> lk(mtx_);
  cb_ = std::move(cb);
}

uint64_t Chassis::unix_ms()
{
  using namespace std::chrono;
  return static_cast<uint64_t>(
    duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count());
}

Geometry Chassis::effective_geometry(Clock::time_point now) const
{
  Geometry g = opt_.geometry;
  const bool fresh = state_.limits_stamp && seconds(now - *state_.limits_stamp) <= kLiveLimitMaxAge;
  if (opt_.protocol_type == Protocol::kLci && opt_.follow_live_limits && fresh) {
    const double live_vx = cmd_.twist.vx >= 0.0 ? state_.limits.max_vx_fwd : state_.limits.max_vx_rev;
    g.max_vx = std::min(g.max_vx, live_vx);
    if (state_.limits.max_steer_rad > 0.0) {
      g.max_steer_rad = std::min(g.max_steer_rad, state_.limits.max_steer_rad);
    }
  }
  return g;
}

std::vector<CanFrame> Chassis::encode_locked(
  const MotionCommand & out, const SteerSolution & sol, bool drive, Clock::time_point now, double dt)
{
  switch (opt_.protocol_type) {
    case Protocol::kLegacyA: {
        std::optional<uint64_t> t_ms;
        if (opt_.send_host_time) {t_ms = unix_ms();}
        return legacy_a::encode_cycle(out, sol.wheel_rad, sol.front_rad, sol.rear_rad, lamps_,
                 opt_.protocol, t_ms);
      }
    case Protocol::kLci: {
        const bool angles = drive && cmd_explicit_ &&
          (cmd_.front_steer_rad.has_value() || cmd_.rear_steer_rad.has_value());
        std::optional<LampCommand> lamps;
        if (!last_lamps_tx_ || seconds(now - *last_lamps_tx_) >= 0.1 - 1e-3) {
          lamps = lamps_;
          last_lamps_tx_ = now;
        }
        std::optional<uint64_t> t_ms;
        if (opt_.send_host_time && (!last_time_tx_ || seconds(now - *last_time_tx_) >= 1.0 - 1e-3)) {
          t_ms = unix_ms();
          last_time_tx_ = now;
        }
        ++counter_;
        status_.heartbeat_counter = counter_;
        return lci::encode_cycle(out, sol.front_rad, sol.rear_rad,
                 angles ? lci::SteerSource::kAngle : lci::SteerSource::kYawRate, counter_, host_,
                 opt_.lci, lamps, t_ms);
      }
    case Protocol::kLegacyB: {
        // Direction changes only at (near) standstill: brake to a stop first.
        const double measured = state_.motion.twist.vx;
        Gear gear = legacy_b_gear_;
        double target = out.twist.vx;
        const Gear wanted = std::abs(target) < 0.05 ? legacy_b_gear_ :
          (target > 0.0 ? Gear::kDrive : Gear::kReverse);
        if (wanted != legacy_b_gear_) {
          if (std::abs(measured) < 0.3 || legacy_b_gear_ == Gear::kNeutral) {
            gear = wanted;
          } else {
            target = 0.0;
          }
        }
        if (!drive && is_zero(out.twist) && std::abs(measured) < 0.3) {gear = Gear::kNeutral;}
        legacy_b_gear_ = gear;
        // No throttle until the VCU reports the requested gear, and never against the motion.
        const bool gear_confirmed = gear == Gear::kNeutral || state_.steer.gear == gear;
        if (!gear_confirmed || target * measured < 0.0) {target = 0.0;}
        const auto pedals = speed_ctl_.step(target, measured, dt, opt_.legacy_b);
        status_.pedals = pedals;
        legacy_b_angle_ = sol.front_rad;
        const Gear wire_gear = out.gear == Gear::kPark ? Gear::kPark : gear;
        const double cap = std::abs(target) < 0.05 ? 0.0 : std::abs(target) * 1.1 + 0.3;
        ++counter_;
        status_.heartbeat_counter = counter_;
        return {legacy_b::encode_command(pedals, sol.front_rad, cap, wire_gear, host_, lamps_,
            counter_, opt_.legacy_b)};
      }
  }
  return {};
}

void Chassis::tick(Clock::time_point now)
{
  std::vector<CanFrame> frames;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    const double dt = last_tick_ ? seconds(now - *last_tick_) : 1.0 / opt_.tx_rate_hz;
    last_tick_ = now;

    status_.command_age_s = cmd_stamp_ ? seconds(now - *cmd_stamp_) : -1.0;
    status_.command_active = cmd_stamp_ && status_.command_age_s <= opt_.cmd_timeout_s;
    status_.feedback_age_s = state_.motion_stamp ? seconds(now - *state_.motion_stamp) : -1.0;
    status_.feedback_ok = state_.motion_stamp && status_.feedback_age_s <= opt_.feedback_timeout_s;
    status_.host = host_;
    if (state_.version_stamp) {
      status_.model_mismatch = state_.version.model_id != lci::model_id_for(opt_.geometry.build);
    }

    const Geometry g = effective_geometry(now);
    status_.live_limits_applied = g.max_vx != opt_.geometry.max_vx ||
      g.max_steer_rad != opt_.geometry.max_steer_rad;

    const bool drive = status_.command_active && status_.feedback_ok;
    const Twist2D target = drive ? cmd_.twist : Twist2D{};
    const Twist2D ramped = slew_.step(target, dt);
    SteerSolution sol = solve(ramped, cmd_.model, g);
    if (drive && cmd_explicit_) {
      if (cmd_.front_steer_rad) {sol.front_rad = std::clamp(*cmd_.front_steer_rad,
            -g.max_steer_rad, g.max_steer_rad);}
      if (cmd_.rear_steer_rad) {sol.rear_rad = std::clamp(*cmd_.rear_steer_rad,
            -g.max_steer_rad, g.max_steer_rad);}
      if (cmd_.wheel_steer_rad) {sol.wheel_rad = *cmd_.wheel_steer_rad;}
    }
    // The request is "limited" if solve() reduced it (compare against the ramped value).
    status_.limited = drive && solve(cmd_.twist, cmd_.model, g).limited;

    MotionCommand out;
    out.twist = sol.twist;
    out.model = sol.model;
    if (drive || !is_zero(sol.twist)) {
      out.gear = cmd_.gear;
    } else {
      out.gear = opt_.park_when_idle ? Gear::kPark : Gear::kNeutral;
    }
    status_.model = out.model;
    status_.gear = out.gear;
    status_.last = sol;

    frames = encode_locked(out, sol, drive, now, dt);
    ++status_.tx_cycles;
  }
  uint64_t errors = 0;
  for (const auto & f : frames) {
    if (!can_ || !can_->write(f)) {++errors;}
  }
  if (errors) {
    std::lock_guard<std::mutex> lk(mtx_);
    status_.tx_errors += errors;
    if (can_) {error_ = can_->last_error();}
  }
}

void Chassis::tx_loop()
{
  const auto period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / opt_.tx_rate_hz));
  auto next = Clock::now();
  while (running_) {
    tick(Clock::now());
    next += period;
    const auto now = Clock::now();
    if (next < now) {next = now;}   // never try to catch up with a burst
    std::this_thread::sleep_until(next);
  }
}

bool Chassis::process(const CanFrame & frame, Clock::time_point stamp)
{
  FeedbackCallback cb;
  ChassisState snapshot;
  bool ok = false;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    switch (opt_.protocol_type) {
      case Protocol::kLegacyA:
        ok = legacy_a::decode(frame, opt_.protocol, state_, stamp);
        break;
      case Protocol::kLci:
        ok = lci::decode(frame, opt_.lci, state_, stamp);
        break;
      case Protocol::kLegacyB:
        ok = legacy_b::decode(frame, opt_.legacy_b, state_, stamp);
        if (ok) {
          // No yaw rate or steering feedback on this interface: use the commanded axle angle.
          const double L = opt_.geometry.wheelbase;
          const double vx = state_.motion.twist.vx;
          const double k = cmd_.model == MotionModel::kDualAckermann ? 2.0 : 1.0;
          state_.motion.twist.wz = k * vx * std::tan(legacy_b_angle_) / L;
          state_.motion.twist.vy = cmd_.model == MotionModel::kFrontAckermann ?
            state_.motion.twist.wz * L / 2.0 : 0.0;
          state_.motion.model_raw = static_cast<uint8_t>(cmd_.model);
          state_.steer.front_rad = legacy_b_angle_;
          state_.steer.rear_rad = cmd_.model == MotionModel::kDualAckermann ? -legacy_b_angle_ : 0.0;
          state_.steer_stamp = stamp;
        }
        break;
    }
    if (ok && cb_) {
      cb = cb_;
      snapshot = state_;
    }
  }
  if (cb) {cb(snapshot, frame.id);}
  return ok;
}

void Chassis::rx_loop()
{
  while (running_) {
    if (auto f = can_->read(20)) {process(*f);}
  }
}

}  // namespace luchs_sdk
