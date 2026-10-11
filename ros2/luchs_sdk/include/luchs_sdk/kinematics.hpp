// Copyright 2026 TerraAgent Robotics. Chassis: RJ Tech Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Steering kinematics for the Luchs A builds and Luchs B.
//
// The chassis firmware turns the body twist in 0x5A0 into wheel commands itself. The SDK
// still computes the matching Ackermann and per-wheel angles and sends them in 0x5A1 and the
// wheel-angle frame, so every frame of a control cycle describes the same motion whichever
// of them the firmware reads. It also enforces the steering and speed limits, so a twist the
// chassis cannot follow is reduced, never silently distorted.
//
// Reference point: base_link (centre of the wheelbase, centre of the track).
//   - front Ackermann: the twist is interpreted at the rear axle (bicycle model), as usual for
//     front-steered vehicles; the lateral velocity at base_link follows from the geometry.
//   - dual Ackermann and four-wheel steer: the twist is interpreted at base_link.
// The LCI codec converts between these and the LCI reference point (rear-axle centre), so the
// SDK API is the same on every Luchs chassis.

#ifndef LUCHS_SDK__KINEMATICS_HPP_
#define LUCHS_SDK__KINEMATICS_HPP_

#include "luchs_sdk/types.hpp"

namespace luchs_sdk
{

struct Geometry
{
  Build build{Build::kFourWheelSteer};
  double wheelbase{0.594};      ///< m, front axle to rear axle
  double track{0.500};          ///< m, 0.500 for A-4WS, 0.596 for A-DF / A-FA / A-DA, 0.970 for B
  double wheel_radius{0.103};   ///< m, 206 mm wheels (A); 22x7-10 tyres, 0.279 m (B)
  double max_steer_rad{1.5708}; ///< per wheel / per axle
  double max_vx{2.0};           ///< m/s   (vehicle maximum 11.6 km/h = 3.22 m/s)
  double max_vy{1.0};           ///< m/s   (four-wheel-steer build only)
  double max_wz{1.5};           ///< rad/s
  double pivot_vx_threshold{0.02};  ///< |vx| below this + yaw request -> turn on the spot
  bool allow_pivot_turn{true};      ///< Ackermann builds turn on the spot in differential mode

  static Geometry for_build(Build b);
};

/// Wheel positions (x, y) relative to base_link, SDK wheel order.
std::array<std::array<double, 2>, 4> wheel_positions(const Geometry & g);

/// Per-wheel steering angles (SDK wheel order) that match the measured axle (bicycle) angles,
/// assuming all wheels roll about one instantaneous centre. For joint states and displays.
PerWheel axle_to_wheel_angles(double front_rad, double rear_rad, const Geometry & g);

/// Motion models the firmware accepts for a build. The first one is the default.
bool model_supported(Build build, MotionModel model);
MotionModel default_model(Build build);

struct SteerSolution
{
  Twist2D twist;               ///< twist actually commanded (after limits)
  MotionModel model{MotionModel::kDifferential};   ///< may switch to differential for pivots
  double front_rad{0.0};       ///< Ackermann (bicycle) front angle
  double rear_rad{0.0};        ///< Ackermann (bicycle) rear angle
  PerWheel wheel_rad{};        ///< per-wheel steering angle
  PerWheel wheel_speed{};      ///< per-wheel ground speed, m/s (signed)
  bool limited{false};         ///< true if the request was reduced
};

/// Turns a requested body twist into a consistent, limited set of chassis commands.
SteerSolution solve(const Twist2D & request, MotionModel model, const Geometry & g);

/// Acceleration limiter for the commanded twist.
class SlewLimiter
{
public:
  SlewLimiter(double max_ax, double max_ay, double max_awz)
  : ax_(max_ax), ay_(max_ay), awz_(max_awz) {}
  Twist2D step(const Twist2D & target, double dt);
  void reset(const Twist2D & t = {}) {cur_ = t;}
  const Twist2D & current() const {return cur_;}

private:
  double ax_, ay_, awz_;
  Twist2D cur_{};
};

/// Planar dead reckoning from body twist (midpoint integration).
class Odometry
{
public:
  void update(const Twist2D & t, double dt);
  void reset() {x_ = y_ = yaw_ = 0.0;}
  double x() const {return x_;}
  double y() const {return y_;}
  double yaw() const {return yaw_;}

private:
  double x_{0.0}, y_{0.0}, yaw_{0.0};
};

}  // namespace luchs_sdk

#endif  // LUCHS_SDK__KINEMATICS_HPP_
