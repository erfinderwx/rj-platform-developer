// Copyright 2026 TerraAgent Robotics. Chassis: RJ Tech Limited.
// SPDX-License-Identifier: Apache-2.0

#include "luchs_sdk/kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace luchs_sdk
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = kPi / 2.0;

double clampAbs(double v, double lim, bool & limited)
{
  if (std::abs(v) > lim) {
    limited = true;
    return std::copysign(lim, v);
  }
  return v;
}

// Rigid-body velocity of each wheel contact for a twist given at (x_ref, 0).
void wheel_vectors(
  const Twist2D & t, double x_ref, const Geometry & g, PerWheel & angle, PerWheel & speed)
{
  const auto pos = wheel_positions(g);
  for (std::size_t i = 0; i < 4; ++i) {
    const double vx = t.vx - t.wz * pos[i][1];
    const double vy = t.vy + t.wz * (pos[i][0] - x_ref);
    double a = std::atan2(vy, vx);
    double s = std::hypot(vx, vy);
    if (s < 1e-9) {a = 0.0;}
    // keep the steering angle within +-90 deg and drive the wheel backwards instead
    if (a > kHalfPi) {a -= kPi; s = -s;} else if (a < -kHalfPi) {a += kPi; s = -s;}
    angle[i] = a;
    speed[i] = s;
  }
}

double reference_x(MotionModel m, const Geometry & g)
{
  return m == MotionModel::kFrontAckermann ? -g.wheelbase / 2.0 : 0.0;
}

bool steerable(std::size_t wheel, MotionModel m)
{
  switch (m) {
    case MotionModel::kDifferential: return false;
    case MotionModel::kFrontAckermann: return wheel == kFrontLeft || wheel == kFrontRight;
    case MotionModel::kDualAckermann:
    case MotionModel::kFourWheelSteer: return true;
  }
  return false;
}

bool within_limits(const Twist2D & t, MotionModel m, const Geometry & g)
{
  PerWheel a{}, s{};
  wheel_vectors(t, reference_x(m, g), g, a, s);
  for (std::size_t i = 0; i < 4; ++i) {
    if (steerable(i, m) && std::abs(a[i]) > g.max_steer_rad + 1e-9) {return false;}
  }
  return true;
}
}  // namespace

Geometry Geometry::for_build(Build b)
{
  Geometry g;
  g.build = b;
  switch (b) {
    case Build::kSkidSteer: g.track = 0.596; g.max_steer_rad = 0.0; break;
    case Build::kFrontAckermann: g.track = 0.596; g.max_steer_rad = 0.60; break;   // TO CONFIRM
    case Build::kDualAckermann: g.track = 0.596; g.max_steer_rad = 0.60; break;    // TO CONFIRM
    case Build::kFourWheelSteer: g.track = 0.500; g.max_steer_rad = kHalfPi; break; // TO CONFIRM
    case Build::kLuchsB:
      // Data sheet v1.3: wheelbase 1430 mm (chassis team: 1445 mm, TO CONFIRM), track 970 mm
      // (top view: 990 mm), 22x7-10 tyres. Max road-wheel angle 30 deg (TO CONFIRM; the VCU
      // upper interface allows 42 deg). Turning on the spot: not available (TO CONFIRM).
      g.wheelbase = 1.430;
      g.track = 0.970;
      g.wheel_radius = 0.2794;
      g.max_steer_rad = 0.5236;
      g.max_vx = 2.0;         // software default for first drives; vehicle maximum 40 km/h
      g.max_vy = 0.0;
      g.max_wz = 1.0;
      g.allow_pivot_turn = false;
      break;
  }
  return g;
}

PerWheel axle_to_wheel_angles(double front, double rear, const Geometry & g)
{
  PerWheel out{};
  const auto pos = wheel_positions(g);
  const double tf = std::tan(std::clamp(front, -1.5, 1.5));
  const double tr = std::tan(std::clamp(rear, -1.5, 1.5));
  if (std::abs(tf - tr) < 1e-9) {          // straight or crab: every wheel at the same angle
    out = {front, front, rear, rear};
    return out;
  }
  // Instantaneous centre (x0, y0): tan(front) = (L/2 - x0) / y0, tan(rear) = (-L/2 - x0) / y0
  const double y0 = g.wheelbase / (tf - tr);
  const double x0 = g.wheelbase / 2.0 - y0 * tf;
  for (std::size_t i = 0; i < 4; ++i) {
    out[i] = std::atan((pos[i][0] - x0) / (y0 - pos[i][1]));
  }
  return out;
}

std::array<std::array<double, 2>, 4> wheel_positions(const Geometry & g)
{
  const double hx = g.wheelbase / 2.0, hy = g.track / 2.0;
  return {{{hx, hy}, {hx, -hy}, {-hx, hy}, {-hx, -hy}}};
}

bool model_supported(Build b, MotionModel m)
{
  switch (b) {
    case Build::kSkidSteer: return m == MotionModel::kDifferential;
    case Build::kFrontAckermann:
      return m == MotionModel::kFrontAckermann || m == MotionModel::kDifferential;
    case Build::kDualAckermann:
      return m == MotionModel::kDualAckermann || m == MotionModel::kFrontAckermann ||
             m == MotionModel::kDifferential;
    case Build::kFourWheelSteer: return true;
    case Build::kLuchsB:
      return m == MotionModel::kDualAckermann || m == MotionModel::kFrontAckermann;
  }
  return false;
}

MotionModel default_model(Build b)
{
  switch (b) {
    case Build::kSkidSteer: return MotionModel::kDifferential;
    case Build::kFrontAckermann: return MotionModel::kFrontAckermann;
    case Build::kDualAckermann: return MotionModel::kDualAckermann;
    case Build::kFourWheelSteer: return MotionModel::kFourWheelSteer;
    case Build::kLuchsB: return MotionModel::kDualAckermann;
  }
  return MotionModel::kDifferential;
}

SteerSolution solve(const Twist2D & req, MotionModel model, const Geometry & g)
{
  SteerSolution out;
  out.model = model;
  Twist2D t = req;
  if (!std::isfinite(t.vx) || !std::isfinite(t.vy) || !std::isfinite(t.wz)) {
    t = Twist2D{};
    out.limited = true;
  }
  const double vx_req = t.vx;
  t.vx = clampAbs(t.vx, g.max_vx, out.limited);
  // Ackermann models: a speed reduced by a limit keeps its path curvature (wz / vx).
  const bool ackermann_req = model == MotionModel::kFrontAckermann ||
    model == MotionModel::kDualAckermann;
  if (ackermann_req && t.vx != vx_req && std::abs(vx_req) > 1e-9) {
    t.wz *= t.vx / vx_req;
  }
  t.wz = clampAbs(t.wz, g.max_wz, out.limited);
  if (model == MotionModel::kFourWheelSteer) {
    t.vy = clampAbs(t.vy, g.max_vy, out.limited);
  } else if (t.vy != 0.0) {
    t.vy = 0.0;               // only the four-wheel-steer model moves sideways
    out.limited = true;
  }

  // Turn on the spot: Ackermann builds use the differential model for a pure yaw request.
  const bool ackermann = model == MotionModel::kFrontAckermann ||
    model == MotionModel::kDualAckermann;
  if (ackermann && std::abs(t.vx) < g.pivot_vx_threshold && std::abs(t.wz) > 1e-6) {
    if (g.allow_pivot_turn) {
      out.model = MotionModel::kDifferential;
      t.vx = 0.0;
    } else {
      t.wz = 0.0;
      out.limited = true;
    }
  }

  // Steering limit: keep vx, scale the turning part (vy, wz) until every steered wheel fits.
  if (out.model != MotionModel::kDifferential && !within_limits(t, out.model, g)) {
    double lo = 0.0, hi = 1.0;
    for (int i = 0; i < 30; ++i) {
      const double mid = 0.5 * (lo + hi);
      Twist2D s{t.vx, t.vy * mid, t.wz * mid};
      (within_limits(s, out.model, g) ? lo : hi) = mid;
    }
    t.vy *= lo;
    t.wz *= lo;
    out.limited = true;
  }

  out.twist = t;
  wheel_vectors(t, reference_x(out.model, g), g, out.wheel_rad, out.wheel_speed);
  for (std::size_t i = 0; i < 4; ++i) {
    if (!steerable(i, out.model)) {out.wheel_rad[i] = 0.0;}
  }
  if (out.model == MotionModel::kDifferential) {
    const auto pos = wheel_positions(g);
    for (std::size_t i = 0; i < 4; ++i) {out.wheel_speed[i] = t.vx - t.wz * pos[i][1];}
  }

  // Bicycle-model axle angles for 0x5A1.
  if (out.model == MotionModel::kFrontAckermann && std::abs(t.vx) > 1e-9) {
    out.front_rad = std::atan(g.wheelbase * t.wz / t.vx);
  } else if (out.model == MotionModel::kDualAckermann && std::abs(t.vx) > 1e-9) {
    out.front_rad = std::atan(g.wheelbase * t.wz / (2.0 * t.vx));
    out.rear_rad = -out.front_rad;
  } else if (out.model == MotionModel::kFourWheelSteer) {
    out.front_rad = 0.5 * (out.wheel_rad[kFrontLeft] + out.wheel_rad[kFrontRight]);
    out.rear_rad = 0.5 * (out.wheel_rad[kRearLeft] + out.wheel_rad[kRearRight]);
  }
  return out;
}

Twist2D SlewLimiter::step(const Twist2D & target, double dt)
{
  auto approach = [dt](double cur, double tgt, double rate) {
      if (rate <= 0.0 || dt <= 0.0) {return tgt;}
      const double d = std::clamp(tgt - cur, -rate * dt, rate * dt);
      return cur + d;
    };
  cur_.vx = approach(cur_.vx, target.vx, ax_);
  cur_.vy = approach(cur_.vy, target.vy, ay_);
  cur_.wz = approach(cur_.wz, target.wz, awz_);
  return cur_;
}

void Odometry::update(const Twist2D & t, double dt)
{
  if (!(dt > 0.0) || dt > 1.0) {return;}
  const double mid = yaw_ + 0.5 * t.wz * dt;
  x_ += (t.vx * std::cos(mid) - t.vy * std::sin(mid)) * dt;
  y_ += (t.vx * std::sin(mid) + t.vy * std::cos(mid)) * dt;
  yaw_ = std::remainder(yaw_ + t.wz * dt, 2.0 * kPi);
}

}  // namespace luchs_sdk
