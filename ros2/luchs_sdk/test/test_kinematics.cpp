// Copyright 2026 TerraAgent Robotics. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>

#include "luchs_sdk/kinematics.hpp"

using namespace luchs_sdk;

TEST(Kinematics, FourWheelSteerCrab)
{
  auto g = Geometry::for_build(Build::kFourWheelSteer);
  auto s = solve({0.3, 0.3, 0.0}, MotionModel::kFourWheelSteer, g);
  for (double a : s.wheel_rad) {EXPECT_NEAR(a, M_PI / 4, 1e-9);}
  EXPECT_FALSE(s.limited);
}

TEST(Kinematics, FourWheelSteerSpinInPlace)
{
  auto g = Geometry::for_build(Build::kFourWheelSteer);
  auto s = solve({0.0, 0.0, 0.5}, MotionModel::kFourWheelSteer, g);
  // tangent direction is 90 deg + atan2(0.25, 0.297) from x; folded into +-90 deg it has
  // magnitude atan2(0.297, 0.25), and the wheel then drives backwards
  const double a = std::atan2(0.297, 0.25);
  EXPECT_NEAR(std::abs(s.wheel_rad[kFrontLeft]), a, 1e-9);
  EXPECT_NEAR(s.wheel_rad[kFrontLeft], s.wheel_rad[kRearRight], 1e-9);
  EXPECT_NEAR(s.wheel_rad[kFrontRight], -s.wheel_rad[kFrontLeft], 1e-9);
  // all wheels on the same circle: same speed magnitude
  for (double v : s.wheel_speed) {EXPECT_NEAR(std::abs(v), 0.5 * std::hypot(0.297, 0.25), 1e-9);}
}

TEST(Kinematics, DualAckermannBicycleAngles)
{
  auto g = Geometry::for_build(Build::kDualAckermann);
  auto s = solve({1.0, 0.0, 0.5}, MotionModel::kDualAckermann, g);
  EXPECT_NEAR(s.front_rad, std::atan(0.594 * 0.5 / 2.0), 1e-9);
  EXPECT_NEAR(s.rear_rad, -s.front_rad, 1e-12);
  EXPECT_GT(s.wheel_rad[kFrontLeft], s.wheel_rad[kFrontRight]);   // inner wheel steers more
}

TEST(Kinematics, FrontAckermannLimitsSteering)
{
  auto g = Geometry::for_build(Build::kFrontAckermann);
  auto s = solve({0.5, 0.0, 2.0}, MotionModel::kFrontAckermann, g);
  EXPECT_TRUE(s.limited);
  for (double a : s.wheel_rad) {EXPECT_LE(std::abs(a), g.max_steer_rad + 1e-6);}
  EXPECT_DOUBLE_EQ(s.wheel_rad[kRearLeft], 0.0);
  EXPECT_DOUBLE_EQ(s.twist.vx, 0.5);    // speed kept, turn rate reduced
  EXPECT_LT(s.twist.wz, 2.0);
}

TEST(Kinematics, AckermannPivotUsesDifferentialModel)
{
  auto g = Geometry::for_build(Build::kFrontAckermann);
  auto s = solve({0.0, 0.0, 0.4}, MotionModel::kFrontAckermann, g);
  EXPECT_EQ(s.model, MotionModel::kDifferential);
  for (double a : s.wheel_rad) {EXPECT_DOUBLE_EQ(a, 0.0);}
  g.allow_pivot_turn = false;
  s = solve({0.0, 0.0, 0.4}, MotionModel::kFrontAckermann, g);
  EXPECT_DOUBLE_EQ(s.twist.wz, 0.0);
}

TEST(Kinematics, SpeedLimitsAndNoSidewaysOutside4ws)
{
  auto g = Geometry::for_build(Build::kSkidSteer);
  auto s = solve({5.0, 0.4, -9.0}, MotionModel::kDifferential, g);
  EXPECT_TRUE(s.limited);
  EXPECT_DOUBLE_EQ(s.twist.vx, g.max_vx);
  EXPECT_DOUBLE_EQ(s.twist.vy, 0.0);
  EXPECT_DOUBLE_EQ(s.twist.wz, -g.max_wz);
  EXPECT_FALSE(model_supported(Build::kSkidSteer, MotionModel::kFourWheelSteer));
}

TEST(Kinematics, SlewAndOdometry)
{
  SlewLimiter lim(1.0, 1.0, 2.0);
  auto t = lim.step({1.0, 0.0, 1.0}, 0.1);
  EXPECT_NEAR(t.vx, 0.1, 1e-12);
  EXPECT_NEAR(t.wz, 0.2, 1e-12);
  Odometry o;
  for (int i = 0; i < 100; ++i) {o.update({1.0, 0.0, 0.0}, 0.01);}
  EXPECT_NEAR(o.x(), 1.0, 1e-9);
  o.reset();
  for (int i = 0; i < 1000; ++i) {o.update({0.0, 0.0, M_PI / 2}, 0.001);}
  EXPECT_NEAR(o.yaw(), M_PI / 2, 1e-9);
}
