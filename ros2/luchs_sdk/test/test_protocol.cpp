// Copyright 2026 TerraAgent Robotics. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>

#include "luchs_sdk/legacy_a_protocol.hpp"

using namespace luchs_sdk;
using namespace luchs_sdk::legacy_a;

static std::array<uint8_t, 8> B(std::initializer_list<int> v)
{
  std::array<uint8_t, 8> a{};
  std::size_t i = 0;
  for (int x : v) {a[i++] = static_cast<uint8_t>(x);}
  return a;
}

TEST(LegacyA, MotionCommandBytes)
{
  auto f = encode_motion({0.5, -0.25, 1.0}, MotionModel::kFourWheelSteer);
  EXPECT_EQ(f.id, 0x5A0u);
  EXPECT_EQ(f.dlc, 8);
  EXPECT_EQ(f.data, B({0x01, 0xF4, 0xFF, 0x06, 0x03, 0xE8, 0x03, 0x00}));
}

TEST(LegacyA, SteerCommandBytes)
{
  auto f = encode_steer(Gear::kDrive, 0.3, -0.3);
  EXPECT_EQ(f.id, 0x5A1u);
  EXPECT_EQ(f.data, B({0x04, 0x01, 0x2C, 0xFE, 0xD4, 0, 0, 0}));
}

TEST(LegacyA, WheelSteerDefaultIsReferenceDriverLayout)
{
  Config cfg;
  auto f = encode_wheel_steer({0.1, 0.2, 0.3, 0.4}, cfg);   // LF RF LR RR
  EXPECT_EQ(f.id, 0x5A3u);
  EXPECT_EQ(f.data, B({0x00, 0x64, 0x00, 0xC8, 0x01, 0x2C, 0x01, 0x90}));
  EXPECT_EQ(encode_lamps({TurnSignal::kLeft, true, false}, cfg).id, 0x5A2u);
}

TEST(LegacyA, WheelSteerDocumentLayout)
{
  Config cfg;
  cfg.wheel_steer_cmd_id = kId5A2;
  cfg.wheel_steer_cmd_order = WheelOrder::kLfLrRfRr;
  auto f = encode_wheel_steer({0.1, 0.2, 0.3, 0.4}, cfg);
  EXPECT_EQ(f.id, 0x5A2u);
  EXPECT_EQ(f.data, B({0x00, 0x64, 0x01, 0x2C, 0x00, 0xC8, 0x01, 0x90}));
  auto l = encode_lamps({TurnSignal::kRight, false, true}, cfg);
  EXPECT_EQ(l.id, 0x5A3u);
  EXPECT_EQ(l.data, B({0x02, 0x00, 0x01, 0, 0, 0, 0, 0}));
}

TEST(LegacyA, HostTimeIsBigEndian64)
{
  auto f = encode_host_time(0x0000019A3B2C1D0EULL);
  EXPECT_EQ(f.id, 0x5A4u);
  EXPECT_EQ(f.data, B({0x00, 0x00, 0x01, 0x9A, 0x3B, 0x2C, 0x1D, 0x0E}));
}

TEST(LegacyA, Saturation)
{
  EXPECT_EQ(to_raw16(40.0), 32767);
  EXPECT_EQ(to_raw16(-40.0), -32768);
  EXPECT_EQ(to_raw16(std::nan("")), 0);
  EXPECT_EQ(to_raw16(0.0015), 2);    // rounds, does not truncate
}

TEST(LegacyA, DecodeMotionAndSource)
{
  Config cfg;
  ChassisState s;
  CanFrame f{0x5B5, 8, B({0xFF, 0x38, 0x00, 0x64, 0x01, 0xF4, 0x03, 0x22})};
  ASSERT_TRUE(decode(f, cfg, s));
  EXPECT_DOUBLE_EQ(s.motion.twist.vx, -0.2);
  EXPECT_DOUBLE_EQ(s.motion.twist.vy, 0.1);
  EXPECT_DOUBLE_EQ(s.motion.twist.wz, 0.5);
  EXPECT_EQ(s.motion.source, ControlSource::kHost);
  f.data[7] = 11;
  decode(f, cfg, s);
  EXPECT_EQ(s.motion.source, ControlSource::kRemoteControl);
  EXPECT_TRUE(s.motion_stamp.has_value());
}

TEST(LegacyA, DecodeWheelSteerOrder)
{
  Config cfg;    // 0x5B7 order LF, LR, RF, RR
  ChassisState s;
  CanFrame f{0x5B7, 8, B({0x00, 0x64, 0x00, 0xC8, 0x01, 0x2C, 0x01, 0x90})};
  ASSERT_TRUE(decode(f, cfg, s));
  EXPECT_DOUBLE_EQ(s.wheel_steer_rad[kFrontLeft], 0.1);
  EXPECT_DOUBLE_EQ(s.wheel_steer_rad[kRearLeft], 0.2);
  EXPECT_DOUBLE_EQ(s.wheel_steer_rad[kFrontRight], 0.3);
  EXPECT_DOUBLE_EQ(s.wheel_steer_rad[kRearRight], 0.4);
}

TEST(LegacyA, DecodeMotorFaultUsesFullBytes)
{
  Config cfg;
  ChassisState s;
  CanFrame f{0x5BA, 8, B({0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0})};
  ASSERT_TRUE(decode(f, cfg, s));
  EXPECT_EQ(s.motor_fault_high, 0x12345678u);
  EXPECT_EQ(s.motor_fault_low, 0x9ABCDEF0u);
}

TEST(LegacyA, DecodeBattery)
{
  Config cfg;
  ChassisState s;
  // SoC raw 875 -> 87.5 %, temp raw 253 -> 25.3 C, charging 0, voltage raw 516 -> 51.6 V
  CanFrame f{0x5BB, 8, B({0x03, 0x6B, 0x00, 0xFD, 0x00, 0x00, 0x02, 0x04})};
  ASSERT_TRUE(decode(f, cfg, s));
  EXPECT_NEAR(s.battery.soc_percent, 87.5, 1e-9);
  EXPECT_NEAR(s.battery.temperature_c, 25.3, 1e-9);
  EXPECT_NEAR(s.battery.voltage_v, 51.6, 1e-9);
  EXPECT_EQ(s.battery.raw[3], 516);
}

TEST(LegacyA, DecodeProtectionAndLamps)
{
  Config cfg;
  ChassisState s;
  ASSERT_TRUE(decode({0x5B8, 8, B({3, 1, 2, 0x02, 120, 255, 0, 45})}, cfg, s));
  EXPECT_EQ(s.safety.fault_code, 0x02);
  EXPECT_NEAR(s.safety.ultrasonic_m[0], 1.20, 1e-9);
  EXPECT_NEAR(s.safety.ultrasonic_m[1], 2.55, 1e-9);
  EXPECT_EQ(fault_code_text(0x0D), "remote-control link lost");
  ASSERT_TRUE(decode({0x5BC, 8, B({1, 0, 1, 0x11, 0, 0, 0, 0})}, cfg, s));
  EXPECT_EQ(s.lamps.light_rain_raw, 0x11);
}

TEST(LegacyA, RejectsUnknownAndShortFrames)
{
  Config cfg;
  ChassisState s;
  EXPECT_FALSE(decode({0x5A0, 8, {}}, cfg, s));     // our own command echoed
  EXPECT_FALSE(decode({0x5B5, 4, {}}, cfg, s));
  EXPECT_EQ(s.frames_unknown, 2u);
  EXPECT_EQ(s.frames_received, 0u);
}

TEST(LegacyA, CycleHasFiveFramesWithTime)
{
  MotionCommand c;
  auto v = encode_cycle(c, {}, 0, 0, {}, Config{}, 1234);
  ASSERT_EQ(v.size(), 5u);
  EXPECT_EQ(v[0].id, 0x5A0u);
  EXPECT_EQ(v[4].id, 0x5A4u);
  EXPECT_EQ(encode_cycle(c, {}, 0, 0, {}, Config{}).size(), 4u);
}
