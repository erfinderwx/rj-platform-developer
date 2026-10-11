// Copyright 2026 TerraAgent Robotics. SPDX-License-Identifier: Apache-2.0
//
// LCI v1.0 draft r3 codec. The byte vectors marked "DBC vector" are also checked against
// Luchs_B_LCI_v1.0_draft.dbc with cantools by dbc/check_lci_dbc.py, so the codec and the DBC
// handed to the chassis team cannot drift apart.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "luchs_sdk/lci_protocol.hpp"
#include "luchs_sdk/legacy_a_protocol.hpp"

using namespace luchs_sdk;

namespace
{
std::array<uint8_t, 8> bytes(std::initializer_list<int> v)
{
  std::array<uint8_t, 8> d{};
  std::size_t i = 0;
  for (int x : v) {d[i++] = static_cast<uint8_t>(x);}
  return d;
}
lci::Config cfg_b()
{
  lci::Config c;
  c.reference_x = -0.715;
  return c;
}
}  // namespace

TEST(Lci, MotionCommandDualAckermannSendsNoLateralSpeed)   // DBC vector
{
  const auto f = lci::encode_motion({1.0, 0.0, 0.3}, MotionModel::kDualAckermann, cfg_b());
  EXPECT_EQ(f.id, 0x5A0u);
  EXPECT_EQ(f.data, bytes({0x03, 0xE8, 0x00, 0x00, 0x01, 0x2C, 0x04, 0x00}));
}

TEST(Lci, MotionCommandFourWheelSteerConvertsToReferencePoint)
{
  // vy at the rear-axle centre = vy(base_link) + wz * reference_x
  const auto f = lci::encode_motion({0.5, 0.2, 0.4}, MotionModel::kFourWheelSteer, cfg_b());
  EXPECT_EQ(legacy_a::be_i16(f.data[2], f.data[3]), static_cast<int16_t>(std::lround(
      1000.0 * (0.2 - 0.4 * 0.715))));
}

TEST(Lci, SteerCommandLayout)   // DBC vector
{
  const auto f = lci::encode_steer(Gear::kDrive, 0.2113, -0.2113, lci::SteerSource::kAngle, 1.5);
  EXPECT_EQ(f.id, 0x5A1u);
  EXPECT_EQ(f.data, bytes({0x04, 0x00, 0xD3, 0xFF, 0x2D, 0x01, 0x05, 0xDC}));
}

TEST(Lci, LampCommandLayout)   // DBC vector
{
  LampCommand l;
  l.turn = TurnSignal::kHazard;
  l.headlight = 2;
  l.horn = true;
  const auto f = lci::encode_lamps(l);
  EXPECT_EQ(f.id, 0x5A3u);
  EXPECT_EQ(f.data, bytes({0x03, 0x00, 0x00, 0x02, 0x01, 0x00, 0x00, 0x00}));
}

TEST(Lci, HeartbeatBitsSpeedLimitAndChecksum)   // DBC vector
{
  HostRequest r;
  r.park = true;
  r.payload_power_2 = true;
  r.speed_limit = 2.5;
  const auto f = lci::encode_heartbeat(0x7F, r);
  EXPECT_EQ(f.id, 0x5A5u);
  // bit 9 park, bit 12 payload 2 -> byte 1 = 0x12; 2.50 m/s -> 250 = 0x00FA
  // checksum: low byte of (0x7F + 0x12 + 0xFA) = 0x8B, XOR 0xFF = 0x74
  EXPECT_EQ(f.data, bytes({0x7F, 0x12, 0x00, 0xFA, 0x00, 0x00, 0x00, 0x74}));
}

TEST(Lci, EstopRequestIsBit8)
{
  HostRequest r;
  r.estop = true;
  const auto f = lci::encode_heartbeat(0, r);
  EXPECT_EQ(f.data[1], 0x01);
}

TEST(Lci, HostTimeIsBigEndian64On0x5A4)
{
  const auto f = lci::encode_host_time(0x0102030405060708ULL);
  EXPECT_EQ(f.id, 0x5A4u);
  EXPECT_EQ(f.data, bytes({1, 2, 3, 4, 5, 6, 7, 8}));
}

TEST(Lci, CycleCadenceFrames)
{
  MotionCommand c;
  c.model = MotionModel::kDualAckermann;
  auto v = lci::encode_cycle(c, 0, 0, lci::SteerSource::kYawRate, 1, {}, cfg_b());
  ASSERT_EQ(v.size(), 3u);
  EXPECT_EQ(v[0].id, 0x5A0u);
  EXPECT_EQ(v[1].id, 0x5A1u);
  EXPECT_EQ(v[2].id, 0x5A5u);
  v = lci::encode_cycle(c, 0, 0, lci::SteerSource::kYawRate, 1, {}, cfg_b(), LampCommand{}, 1ULL);
  EXPECT_EQ(v.size(), 5u);
}

TEST(Lci, DecodeMotionConvertsToBaseLink)
{
  // dual Ackermann: the rear-axle centre moves sideways, vy = -wz * L / 2; base_link does not.
  ChassisState s;
  const double wz = 0.4;
  CanFrame f{0x5B5, 8, {}};
  legacy_a::put_i16(f.data, 0, legacy_a::to_raw16(1.0));
  legacy_a::put_i16(f.data, 2, legacy_a::to_raw16(-wz * 1.43 / 2.0));
  legacy_a::put_i16(f.data, 4, legacy_a::to_raw16(wz));
  f.data[6] = 0x04;
  f.data[7] = 0x22;
  ASSERT_TRUE(lci::decode(f, cfg_b(), s));
  EXPECT_NEAR(s.motion.twist.vx, 1.0, 1e-9);
  EXPECT_NEAR(s.motion.twist.vy, 0.0, 1e-3);
  EXPECT_NEAR(s.motion.twist.wz, 0.4, 1e-9);
  EXPECT_EQ(s.motion.source, ControlSource::kHost);
  f.data[7] = 0x11;
  lci::decode(f, cfg_b(), s);
  EXPECT_EQ(s.motion.source, ControlSource::kRemoteControl);
}

TEST(Lci, DecodeLimits)   // DBC vector
{
  ChassisState s;
  const CanFrame f{0x5B0, 8, {0x01, 0xF4, 0x00, 0xC8, 0x02, 0x0B, 0x11, 0x03}};
  ASSERT_TRUE(lci::decode(f, cfg_b(), s));
  EXPECT_NEAR(s.limits.max_vx_fwd, 5.00, 1e-9);
  EXPECT_NEAR(s.limits.max_vx_rev, 2.00, 1e-9);
  EXPECT_NEAR(s.limits.max_steer_rad, 0.523, 1e-9);
  EXPECT_EQ(s.limits.reason_bits, 0x11);
  EXPECT_TRUE(s.limits.speed_clipped);
  EXPECT_TRUE(s.limits.steer_clipped);
  EXPECT_FALSE(s.limits.accel_clipped);
  EXPECT_EQ(limit_reason_text(0x11), "slope/roll, host speed limit");
}

TEST(Lci, DecodeChassisStateChecksumAndCounter)   // DBC vector
{
  ChassisState s;
  CanFrame f{0x5BD, 8, {0x02, 0x80, 0x00, 0x0D, 0x01, 0xF4, 0x05, 0x00}};
  // bits: RC online, 4WD, park brake; RCLink 0; speed cap 5.00 m/s; alive 5
  f.data[7] = lci::checksum(f.data);
  ASSERT_TRUE(lci::decode(f, cfg_b(), s));
  EXPECT_EQ(s.status.operating_mode, 2);
  EXPECT_EQ(estop_source_text(s.status.estop_source_bits), "host heartbeat timeout");
  EXPECT_TRUE(s.status.rc_online);
  EXPECT_FALSE(s.status.rc_enabled);
  EXPECT_TRUE(s.status.four_wd);
  EXPECT_TRUE(s.status.park_brake);
  EXPECT_EQ(s.status.rc_link, 0);
  EXPECT_NEAR(s.status.active_speed_limit, 5.0, 1e-9);
  // wrong checksum: ignored and counted
  CanFrame bad = f;
  bad.data[0] = 0x01;
  EXPECT_FALSE(lci::decode(bad, cfg_b(), s));
  EXPECT_EQ(s.status.checksum_errors, 1u);
  EXPECT_EQ(s.status.operating_mode, 2);
  // counter jump is counted
  f.data[6] = 0x09;
  f.data[7] = lci::checksum(f.data);
  ASSERT_TRUE(lci::decode(f, cfg_b(), s));
  EXPECT_EQ(s.status.counter_errors, 1u);
}

TEST(Lci, DecodeRcLinkBits)
{
  ChassisState s;
  CanFrame f{0x5BD, 8, {0x01, 0, 0, 0x23, 0, 0, 0, 0}};   // RC online, RC enabled, link "lost"
  f.data[7] = lci::checksum(f.data);
  ASSERT_TRUE(lci::decode(f, cfg_b(), s));
  EXPECT_EQ(s.status.rc_link, 2);
  EXPECT_TRUE(s.status.rc_enabled);
}

TEST(Lci, DecodeWheelSpeedsBatteryAndProtection)
{
  ChassisState s;
  ASSERT_TRUE(lci::decode({0x5B9, 8, {0x0F, 0xA0, 0x0F, 0xA0, 0xF0, 0x60, 0x00, 0x00}}, cfg_b(), s));
  EXPECT_NEAR(s.wheel_speed[kFrontLeft], 40.0, 1e-9);
  EXPECT_NEAR(s.wheel_speed[kRearLeft], -40.0, 1e-9);
  // SOC 87.50 %, 28.5 C, charge 0, 76.8 V
  ASSERT_TRUE(lci::decode({0x5BB, 8, {0x22, 0x2E, 0x01, 0x1D, 0x00, 0x00, 0x03, 0x00}}, cfg_b(), s));
  EXPECT_NEAR(s.battery.soc_percent, 87.5, 1e-9);
  EXPECT_NEAR(s.battery.temperature_c, 28.5, 1e-9);
  EXPECT_NEAR(s.battery.voltage_v, 76.8, 1e-9);
  ASSERT_TRUE(lci::decode({0x5B8, 8, {0, 1, 0, 0, 120, 255, 255, 255}}, cfg_b(), s));
  EXPECT_EQ(s.safety.bumper_raw, 1);
  EXPECT_NEAR(s.safety.ultrasonic_m[0], 1.2, 1e-9);
  EXPECT_TRUE(std::isinf(s.safety.ultrasonic_m[1]));
}

TEST(Lci, DecodeImuAttitudeBrakeTempsVersion)
{
  ChassisState s;
  ASSERT_TRUE(lci::decode({0x5BE, 8, {0x00, 0x00, 0x00, 0x00, 0x01, 0xF4, 0x07, 0x00}}, cfg_b(), s));
  EXPECT_NEAR(s.imu.rate[2], 0.5, 1e-9);
  EXPECT_EQ(s.imu.rate_counter, 7);
  ASSERT_TRUE(lci::decode({0x5B1, 8, {0x00, 0x00, 0x00, 0x00, 0x13, 0x29, 0x07, 0x00}}, cfg_b(), s));
  EXPECT_NEAR(s.imu.accel[2], 9.81, 1e-9);
  ASSERT_TRUE(lci::decode({0x5B2, 8, {0x03, 0xE8, 0xFC, 0x18, 0, 0, 0, 0}}, cfg_b(), s));
  EXPECT_NEAR(s.imu.roll, 0.1, 1e-9);
  EXPECT_NEAR(s.imu.pitch, -0.1, 1e-9);
  ASSERT_TRUE(lci::decode({0x5C0, 8, {0x00, 0x78, 0x02, 0x00, 0x01, 0x00, 0, 0}}, cfg_b(), s));
  EXPECT_NEAR(s.brake.brake_pressure_mpa, 1.2, 1e-9);
  EXPECT_EQ(s.brake.epb_state, 2);
  EXPECT_EQ(s.brake.eps_rear, 1);
  ASSERT_TRUE(lci::decode({0x5C1, 8, {40, 140, 0, 255, 70, 71, 72, 73}}, cfg_b(), s));
  EXPECT_NEAR(s.temperature.motor[0], 0.0, 1e-9);
  EXPECT_NEAR(s.temperature.motor[1], 100.0, 1e-9);
  EXPECT_NEAR(s.temperature.motor[2], -40.0, 1e-9);
  EXPECT_NEAR(s.temperature.controller[3], 33.0, 1e-9);
  ASSERT_TRUE(lci::decode({0x5BF, 8, {1, 0, 0x1B, 0, 9, 2, 0, 0}}, cfg_b(), s));
  EXPECT_EQ(s.version.model_id, lci::kModelB);
  EXPECT_EQ(model_id_text(s.version.model_id), "Luchs B");
}

TEST(Lci, RejectsCommandsUnknownAndShortFrames)
{
  ChassisState s;
  EXPECT_FALSE(lci::decode({0x5A0, 8, {}}, cfg_b(), s));
  EXPECT_FALSE(lci::decode({0x123, 8, {}}, cfg_b(), s));
  EXPECT_FALSE(lci::decode({0x5B5, 4, {}}, cfg_b(), s));
  EXPECT_EQ(s.frames_received, 0u);
}

TEST(Lci, ModelIds)
{
  EXPECT_EQ(lci::model_id_for(Build::kLuchsB), 0x1B);
  EXPECT_EQ(lci::model_id_for(Build::kFourWheelSteer), 0x0D);
  EXPECT_EQ(lci::model_id_for(Build::kSkidSteer), 0x0A);
}
