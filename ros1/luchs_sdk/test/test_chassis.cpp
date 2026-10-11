// Copyright 2026 TerraAgent Robotics. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <thread>

#include "luchs_sdk/chassis.hpp"
#include "luchs_sdk/sim_bus.hpp"

using namespace luchs_sdk;
using namespace std::chrono_literals;

namespace
{
class FakeBus : public Transport
{
public:
  std::vector<CanFrame> sent;
  bool write(const CanFrame & f) override {sent.push_back(f); return true;}
  std::optional<CanFrame> read(int) override {return std::nullopt;}
  bool is_open() const override {return true;}
  const std::string & last_error() const override {return err;}
  std::string err;
};

CanFrame last_of(const std::vector<CanFrame> & v, uint32_t id)
{
  for (auto it = v.rbegin(); it != v.rend(); ++it) {
    if (it->id == id) {return *it;}
  }
  return {};
}
double vx_of(const CanFrame & f) {return legacy_a::be_i16(f.data[0], f.data[1]) / 1000.0;}
const CanFrame kAlive{0x5B5, 8, {0, 0, 0, 0, 0, 0, 0x03, 0x22}};
}  // namespace

TEST(Chassis, NoFeedbackMeansNoMotion)
{
  auto bus = std::make_unique<FakeBus>();
  auto * raw = bus.get();
  Chassis c(ChassisOptions{}, std::move(bus));
  c.set_twist({0.5, 0, 0});
  auto t = Clock::now();
  for (int i = 0; i < 10; ++i) {c.tick(t += 50ms);}
  EXPECT_DOUBLE_EQ(vx_of(last_of(raw->sent, 0x5A0)), 0.0);
  EXPECT_EQ(last_of(raw->sent, 0x5A1).data[0], static_cast<uint8_t>(Gear::kPark));
  EXPECT_FALSE(c.status().feedback_ok);
}

TEST(Chassis, RampsDrivesThenParksAfterCommandTimeout)
{
  auto bus = std::make_unique<FakeBus>();
  auto * raw = bus.get();
  ChassisOptions o;
  o.max_accel_x = 1.0;
  o.cmd_timeout_s = 0.3;
  o.send_host_time = false;
  Chassis c(o, std::move(bus));
  auto t = Clock::now();
  c.process(kAlive, t);
  c.set_twist({0.5, 0, 0}, t);
  c.tick(t += 50ms);
  // first cycle: ramp limited to 1.0 m/s^2 * 0.05 s
  EXPECT_NEAR(vx_of(last_of(raw->sent, 0x5A0)), 0.05, 1e-3);
  EXPECT_EQ(last_of(raw->sent, 0x5A1).data[0], static_cast<uint8_t>(Gear::kDrive));
  EXPECT_EQ(last_of(raw->sent, 0x5A0).data[6], 0x03);       // 4WS model
  // keep commanding and feeding back for 0.6 s -> reaches 0.5 m/s
  for (int i = 0; i < 12; ++i) {
    c.set_twist({0.5, 0, 0}, t);
    c.process(kAlive, t);
    c.tick(t += 50ms);
  }
  EXPECT_NEAR(vx_of(last_of(raw->sent, 0x5A0)), 0.5, 1e-3);
  // stop commanding: after the timeout it ramps down, then parks
  for (int i = 0; i < 30; ++i) {
    c.process(kAlive, t);
    c.tick(t += 50ms);
  }
  EXPECT_DOUBLE_EQ(vx_of(last_of(raw->sent, 0x5A0)), 0.0);
  EXPECT_EQ(last_of(raw->sent, 0x5A1).data[0], static_cast<uint8_t>(Gear::kPark));
  EXPECT_FALSE(c.status().command_active);
  // every cycle sent 4 frames (no host time)
  EXPECT_EQ(raw->sent.size() % 4, 0u);
}

TEST(Chassis, FeedbackLossStopsMotion)
{
  auto bus = std::make_unique<FakeBus>();
  auto * raw = bus.get();
  ChassisOptions o;
  o.max_accel_x = 100.0;
  o.feedback_timeout_s = 0.2;
  Chassis c(o, std::move(bus));
  auto t = Clock::now();
  c.process(kAlive, t);
  c.set_twist({0.4, 0, 0}, t);
  c.tick(t += 50ms);
  EXPECT_NEAR(vx_of(last_of(raw->sent, 0x5A0)), 0.4, 1e-3);
  for (int i = 0; i < 6; ++i) {
    c.set_twist({0.4, 0, 0}, t);
    c.tick(t += 50ms);      // no feedback any more
  }
  EXPECT_DOUBLE_EQ(vx_of(last_of(raw->sent, 0x5A0)), 0.0);
  EXPECT_FALSE(c.status().feedback_ok);
}

TEST(Chassis, UnsupportedModelIsIgnored)
{
  ChassisOptions o;
  o.geometry = Geometry::for_build(Build::kSkidSteer);
  o.model = MotionModel::kFourWheelSteer;
  Chassis c(o, std::make_unique<FakeBus>());
  EXPECT_EQ(c.status().model, MotionModel::kDifferential);
}

TEST(Chassis, ClosedLoopWithSimBus)
{
  ChassisOptions o;
  o.tx_rate_hz = 50.0;
  Chassis c(o, std::make_unique<SimBus>());
  ASSERT_TRUE(c.start());
  const auto end = Clock::now() + 1500ms;
  while (Clock::now() < end) {
    c.set_twist({0.4, 0.2, 0.0});
    std::this_thread::sleep_for(20ms);
  }
  const auto s = c.state();
  EXPECT_TRUE(c.status().feedback_ok);
  EXPECT_NEAR(s.motion.twist.vx, 0.4, 0.05);
  EXPECT_NEAR(s.motion.twist.vy, 0.2, 0.05);
  EXPECT_NEAR(s.wheel_steer_rad[kFrontLeft], std::atan2(0.2, 0.4), 0.05);
  EXPECT_GT(s.battery.voltage_v, 50.0);
  c.stop();
}
