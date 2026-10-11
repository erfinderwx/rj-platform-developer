// Copyright 2026 TerraAgent Robotics. SPDX-License-Identifier: Apache-2.0
//
// Luchs B: geometry and kinematics, the chassis client on LCI and on the VCU upper interface,
// and end-to-end runs against the simulators.
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

std::vector<CanFrame> of(const std::vector<CanFrame> & v, uint32_t id)
{
  std::vector<CanFrame> out;
  for (const auto & f : v) {if (f.id == id) {out.push_back(f);}}
  return out;
}
double i16(const CanFrame & f, std::size_t at)
{
  return legacy_a::be_i16(f.data[at], f.data[at + 1]) / 1000.0;
}
CanFrame motion_fb(double vx = 0.0)
{
  CanFrame f{0x5B5, 8, {}};
  legacy_a::put_i16(f.data, 0, legacy_a::to_raw16(vx));
  f.data[6] = 0x04;
  f.data[7] = 0x22;
  return f;
}
CanFrame limits_fb(double fwd, double rev, double steer)
{
  CanFrame f{0x5B0, 8, {}};
  auto u16 = [&f](std::size_t at, double v) {
      const auto u = static_cast<uint16_t>(std::lround(v));
      f.data[at] = static_cast<uint8_t>(u >> 8);
      f.data[at + 1] = static_cast<uint8_t>(u & 0xFF);
    };
  u16(0, fwd * 100);
  u16(2, rev * 100);
  u16(4, steer * 1000);
  return f;
}
}  // namespace

// ---- geometry / kinematics ----------------------------------------------------------------

TEST(LuchsB, GeometryAndModels)
{
  const auto g = Geometry::for_build(Build::kLuchsB);
  EXPECT_DOUBLE_EQ(g.wheelbase, 1.430);
  EXPECT_DOUBLE_EQ(g.track, 0.970);
  EXPECT_NEAR(g.max_steer_rad, 30.0 * M_PI / 180.0, 1e-3);
  EXPECT_FALSE(g.allow_pivot_turn);
  EXPECT_EQ(default_model(Build::kLuchsB), MotionModel::kDualAckermann);
  EXPECT_TRUE(model_supported(Build::kLuchsB, MotionModel::kFrontAckermann));
  EXPECT_FALSE(model_supported(Build::kLuchsB, MotionModel::kDifferential));
  EXPECT_FALSE(model_supported(Build::kLuchsB, MotionModel::kFourWheelSteer));
  const auto o = ChassisOptions::for_build(Build::kLuchsB);
  EXPECT_EQ(o.protocol_type, Protocol::kLci);
  EXPECT_DOUBLE_EQ(o.tx_rate_hz, 50.0);
}

TEST(LuchsB, DualAckermannAngleMatchesManualFormula)
{
  const auto g = Geometry::for_build(Build::kLuchsB);
  const auto s = solve({1.0, 0.0, 0.3}, MotionModel::kDualAckermann, g);
  EXPECT_NEAR(s.front_rad, std::atan(1.43 * 0.3 / 2.0), 1e-9);    // manual table 5-1
  EXPECT_NEAR(s.rear_rad, -s.front_rad, 1e-12);
  const auto f = solve({1.0, 0.0, 0.3}, MotionModel::kFrontAckermann, g);
  EXPECT_NEAR(f.front_rad, std::atan(1.43 * 0.3), 1e-9);
  EXPECT_DOUBLE_EQ(f.rear_rad, 0.0);
}

TEST(LuchsB, NoTurnOnTheSpot)
{
  const auto g = Geometry::for_build(Build::kLuchsB);
  const auto s = solve({0.0, 0.0, 0.5}, MotionModel::kDualAckermann, g);
  EXPECT_EQ(s.model, MotionModel::kDualAckermann);
  EXPECT_DOUBLE_EQ(s.twist.wz, 0.0);
  EXPECT_TRUE(s.limited);
}

TEST(LuchsB, SpeedLimitKeepsCurvature)
{
  auto g = Geometry::for_build(Build::kLuchsB);
  g.max_vx = 1.0;
  const auto s = solve({2.0, 0.0, 0.2}, MotionModel::kDualAckermann, g);
  EXPECT_DOUBLE_EQ(s.twist.vx, 1.0);
  EXPECT_NEAR(s.twist.wz, 0.1, 1e-12);       // same radius, 10 m
  EXPECT_TRUE(s.limited);
}

TEST(LuchsB, SteeringLimitScalesTheTurn)
{
  const auto g = Geometry::for_build(Build::kLuchsB);
  const auto s = solve({0.5, 0.0, 1.0}, MotionModel::kFrontAckermann, g);
  EXPECT_TRUE(s.limited);
  for (std::size_t i = 0; i < 2; ++i) {EXPECT_LE(std::abs(s.wheel_rad[i]), g.max_steer_rad + 1e-6);}
  EXPECT_DOUBLE_EQ(s.twist.vx, 0.5);
}

TEST(LuchsB, AxleToWheelAngles)
{
  const auto g = Geometry::for_build(Build::kLuchsB);
  // dual Ackermann: instantaneous centre on the y axis of base_link
  const double f = 0.3;
  const auto w = axle_to_wheel_angles(f, -f, g);
  const double R = (g.wheelbase / 2.0) / std::tan(f);
  EXPECT_NEAR(w[kFrontLeft], std::atan(0.715 / (R - 0.485)), 1e-9);    // inner wheel, larger
  EXPECT_NEAR(w[kFrontRight], std::atan(0.715 / (R + 0.485)), 1e-9);
  EXPECT_NEAR(w[kRearLeft], -w[kFrontLeft], 1e-9);
  // matches the per-wheel solution of the solver
  const auto s = solve({1.0, 0.0, 2.0 * std::tan(f) / g.wheelbase}, MotionModel::kDualAckermann, g);
  for (std::size_t i = 0; i < 4; ++i) {EXPECT_NEAR(w[i], s.wheel_rad[i], 1e-9);}
  // straight
  const auto z = axle_to_wheel_angles(0.0, 0.0, g);
  for (double a : z) {EXPECT_DOUBLE_EQ(a, 0.0);}
}

// ---- chassis client on LCI ----------------------------------------------------------------

TEST(LuchsBLci, HeartbeatEveryCycleWithAdvancingCounter)
{
  auto bus = std::make_unique<FakeBus>();
  auto * raw = bus.get();
  Chassis c(ChassisOptions::for_build(Build::kLuchsB), std::move(bus));
  auto t = Clock::now();
  for (int i = 0; i < 10; ++i) {c.tick(t += 20ms);}
  const auto hb = of(raw->sent, 0x5A5);
  ASSERT_EQ(hb.size(), 10u);
  for (std::size_t i = 1; i < hb.size(); ++i) {
    EXPECT_EQ(hb[i].data[0], static_cast<uint8_t>(hb[i - 1].data[0] + 1));
    EXPECT_EQ(lci::checksum(hb[i].data), hb[i].data[7]);
  }
  // no feedback: zero motion and gear P
  EXPECT_DOUBLE_EQ(i16(of(raw->sent, 0x5A0).back(), 0), 0.0);
  EXPECT_EQ(of(raw->sent, 0x5A1).back().data[0], static_cast<uint8_t>(Gear::kPark));
}

TEST(LuchsBLci, LampsAt10HzAndHostTimeAt1Hz)
{
  auto bus = std::make_unique<FakeBus>();
  auto * raw = bus.get();
  Chassis c(ChassisOptions::for_build(Build::kLuchsB), std::move(bus));
  auto t = Clock::now();
  for (int i = 0; i < 100; ++i) {c.tick(t += 20ms);}   // 2 s at 50 Hz
  EXPECT_EQ(of(raw->sent, 0x5A0).size(), 100u);
  EXPECT_NEAR(static_cast<double>(of(raw->sent, 0x5A3).size()), 20.0, 1.0);
  EXPECT_NEAR(static_cast<double>(of(raw->sent, 0x5A4).size()), 2.0, 1.0);
}

TEST(LuchsBLci, DrivesWithYawRateAndComputedAngles)
{
  auto bus = std::make_unique<FakeBus>();
  auto * raw = bus.get();
  auto o = ChassisOptions::for_build(Build::kLuchsB);
  o.max_accel_x = 100.0;
  o.max_accel_wz = 100.0;
  Chassis c(o, std::move(bus));
  auto t = Clock::now();
  for (int i = 0; i < 5; ++i) {
    c.process(motion_fb(), t);
    c.set_twist({1.0, 0.0, 0.3}, t);
    c.tick(t += 20ms);
  }
  const auto m = of(raw->sent, 0x5A0).back();
  EXPECT_NEAR(i16(m, 0), 1.0, 1e-3);
  EXPECT_NEAR(i16(m, 2), 0.0, 1e-3);           // dual Ackermann: TargetVy = 0
  EXPECT_NEAR(i16(m, 4), 0.3, 1e-3);
  EXPECT_EQ(m.data[6], 0x04);
  const auto s = of(raw->sent, 0x5A1).back();
  EXPECT_EQ(s.data[0], static_cast<uint8_t>(Gear::kDrive));
  EXPECT_NEAR(legacy_a::be_i16(s.data[1], s.data[2]) / 1000.0, std::atan(1.43 * 0.3 / 2.0), 1e-3);
  EXPECT_EQ(s.data[5], 0);                     // SteerSource: yaw rate
}

TEST(LuchsBLci, ExplicitAnglesUseAngleMode)
{
  auto bus = std::make_unique<FakeBus>();
  auto * raw = bus.get();
  Chassis c(ChassisOptions::for_build(Build::kLuchsB), std::move(bus));
  auto t = Clock::now();
  MotionCommand cmd;
  cmd.model = MotionModel::kDualAckermann;
  cmd.front_steer_rad = 0.2;
  cmd.rear_steer_rad = -0.2;
  c.process(motion_fb(), t);
  c.set_command(cmd, t);
  c.tick(t += 20ms);
  const auto s = of(raw->sent, 0x5A1).back();
  EXPECT_EQ(s.data[5], 1);                     // SteerSource: angle (works at standstill)
  EXPECT_NEAR(legacy_a::be_i16(s.data[1], s.data[2]) / 1000.0, 0.2, 1e-3);
  EXPECT_NEAR(legacy_a::be_i16(s.data[3], s.data[4]) / 1000.0, -0.2, 1e-3);
  // beyond the steering limit: clamped
  cmd.front_steer_rad = 1.0;
  c.set_command(cmd, t);
  c.process(motion_fb(), t);
  c.tick(t += 20ms);
  EXPECT_NEAR(legacy_a::be_i16(of(raw->sent, 0x5A1).back().data[1],
    of(raw->sent, 0x5A1).back().data[2]) / 1000.0, 0.5236, 1e-3);
}

TEST(LuchsBLci, LiveLimitsApplyAndKeepCurvature)
{
  auto bus = std::make_unique<FakeBus>();
  auto * raw = bus.get();
  auto o = ChassisOptions::for_build(Build::kLuchsB);
  o.max_accel_x = 100.0;
  o.max_accel_wz = 100.0;
  Chassis c(o, std::move(bus));
  auto t = Clock::now();
  for (int i = 0; i < 5; ++i) {
    c.process(motion_fb(), t);
    c.process(limits_fb(1.0, 0.5, 0.5), t);     // e.g. slope derate to 1 m/s
    c.set_twist({2.0, 0.0, 0.4}, t);
    c.tick(t += 20ms);
  }
  const auto m = of(raw->sent, 0x5A0).back();
  EXPECT_NEAR(i16(m, 0), 1.0, 1e-3);
  EXPECT_NEAR(i16(m, 4), 0.2, 1e-3);          // radius kept: 5 m
  EXPECT_TRUE(c.status().limited);
  EXPECT_TRUE(c.status().live_limits_applied);
  // stale limits (> 200 ms) are not used
  for (int i = 0; i < 20; ++i) {
    c.process(motion_fb(), t);
    c.set_twist({2.0, 0.0, 0.4}, t);
    c.tick(t += 20ms);
  }
  EXPECT_NEAR(i16(of(raw->sent, 0x5A0).back(), 0), 2.0, 1e-3);
}

TEST(LuchsBLci, HostRequestsReachTheHeartbeat)
{
  auto bus = std::make_unique<FakeBus>();
  auto * raw = bus.get();
  Chassis c(ChassisOptions::for_build(Build::kLuchsB), std::move(bus));
  auto t = Clock::now();
  c.request_estop(true);
  c.set_payload_power(1, true);
  c.set_speed_limit(3.0);
  c.tick(t += 20ms);
  const auto hb = of(raw->sent, 0x5A5).back();
  EXPECT_EQ(hb.data[1], 0x01 | 0x08);
  EXPECT_EQ(legacy_a::be_u16(hb.data[2], hb.data[3]), 300);
  EXPECT_TRUE(c.status().host.estop);
}

TEST(LuchsBLci, StopSendsParkRequestAndGearP)
{
  auto bus = std::make_unique<FakeBus>();
  auto * raw = bus.get();
  {
    Chassis c(ChassisOptions::for_build(Build::kLuchsB), std::move(bus));
    ASSERT_TRUE(c.start());
    std::this_thread::sleep_for(60ms);
    c.stop();
    const auto hb = of(raw->sent, 0x5A5);
    ASSERT_GE(hb.size(), 3u);
    for (std::size_t i = hb.size() - 3; i < hb.size(); ++i) {EXPECT_EQ(hb[i].data[1] & 0x02, 0x02);}
    EXPECT_EQ(of(raw->sent, 0x5A1).back().data[0], static_cast<uint8_t>(Gear::kPark));
  }
}

TEST(LuchsBLci, ModelMismatchIsReported)
{
  auto bus = std::make_unique<FakeBus>();
  Chassis c(ChassisOptions::for_build(Build::kLuchsB), std::move(bus));
  auto t = Clock::now();
  c.process({0x5BF, 8, {1, 0, 0x0D, 0, 1, 0, 0, 0}}, t);   // an A-4WS answers
  c.tick(t += 20ms);
  EXPECT_TRUE(c.status().model_mismatch);
  c.process({0x5BF, 8, {1, 0, 0x1B, 0, 1, 0, 0, 0}}, t);
  c.tick(t += 20ms);
  EXPECT_FALSE(c.status().model_mismatch);
}

// ---- end to end against the LCI simulator -------------------------------------------------

TEST(LuchsBSim, LciDrivesTurnsAndReportsBaseLinkMotion)
{
  const auto o = ChassisOptions::for_build(Build::kLuchsB);
  Chassis c(o, make_simulator(o));
  ASSERT_TRUE(c.start());
  const auto end = Clock::now() + 2500ms;
  while (Clock::now() < end) {
    c.set_twist({1.0, 0.0, 0.2});
    std::this_thread::sleep_for(20ms);
  }
  const auto s = c.state();
  const auto st = c.status();
  EXPECT_TRUE(st.feedback_ok);
  EXPECT_NEAR(s.motion.twist.vx, 1.0, 0.1);
  EXPECT_NEAR(s.motion.twist.wz, 0.2, 0.05);
  EXPECT_NEAR(s.motion.twist.vy, 0.0, 0.02);     // dual Ackermann: no slip at base_link
  EXPECT_EQ(s.status.operating_mode, 2);
  EXPECT_EQ(s.version.model_id, lci::kModelB);
  EXPECT_FALSE(st.model_mismatch);
  EXPECT_EQ(s.status.checksum_errors, 0u);
  EXPECT_EQ(s.status.counter_errors, 0u);
  EXPECT_GT(s.limits.max_vx_fwd, 10.0);
  c.stop();
}

TEST(LuchsBSim, LciChassisStopsWhenHeartbeatStops)
{
  // Drive the simulator frame by frame: motion commands keep coming, the heartbeat stops
  // (a hung host process). The chassis must stop, park and report a heartbeat timeout.
  const auto g = Geometry::for_build(Build::kLuchsB);
  SimBusLci sim(g);
  lci::Config cfg;
  cfg.reference_x = -g.wheelbase / 2.0;
  ChassisState st;
  uint8_t n = 0;
  auto run = [&](std::chrono::milliseconds d, bool heartbeat) {
      const auto t0 = Clock::now();
      while (Clock::now() - t0 < d) {
        if (heartbeat) {sim.write(lci::encode_heartbeat(n++, HostRequest{}));}
        sim.write(lci::encode_motion({1.0, 0, 0}, MotionModel::kDualAckermann, cfg));
        sim.write(lci::encode_steer(Gear::kDrive, 0, 0, lci::SteerSource::kYawRate, 0));
        while (auto f = sim.read(0)) {lci::decode(*f, cfg, st);}
        std::this_thread::sleep_for(10ms);
      }
    };
  run(1200ms, true);
  EXPECT_GT(st.motion.twist.vx, 0.5);
  EXPECT_FALSE(st.status.estop_source_bits & 0x80);
  run(1500ms, false);
  EXPECT_NEAR(st.motion.twist.vx, 0.0, 0.05);
  EXPECT_TRUE(st.status.estop_source_bits & 0x80);
  EXPECT_TRUE(st.status.park_brake);
  run(1500ms, true);                           // heartbeat back: drives again
  EXPECT_GT(st.motion.twist.vx, 0.5);
}

TEST(LuchsBSim, LciEstopRequestLatchesUntilMaintenanceReset)
{
  const auto o = ChassisOptions::for_build(Build::kLuchsB);
  auto sim = std::make_unique<SimBusLci>(o.geometry);
  auto * simp = sim.get();
  Chassis c(o, std::move(sim));
  ASSERT_TRUE(c.start());
  auto drive_for = [&c](std::chrono::milliseconds d) {
      const auto end = Clock::now() + d;
      while (Clock::now() < end) {
        c.set_twist({1.0, 0.0, 0.0});
        std::this_thread::sleep_for(20ms);
      }
    };
  drive_for(1200ms);
  EXPECT_GT(c.state().motion.twist.vx, 0.5);
  c.request_estop(true);
  drive_for(1500ms);
  EXPECT_NEAR(c.state().motion.twist.vx, 0.0, 0.05);
  EXPECT_EQ(c.state().status.fault_level, 3);
  c.request_estop(false);             // still latched in the chassis
  drive_for(800ms);
  EXPECT_NEAR(c.state().motion.twist.vx, 0.0, 0.05);
  simp->maintenance_reset();
  drive_for(1500ms);
  EXPECT_GT(c.state().motion.twist.vx, 0.5);
  c.stop();
}

TEST(LuchsBSim, LciRemoteControlHasPriority)
{
  const auto o = ChassisOptions::for_build(Build::kLuchsB);
  auto sim = std::make_unique<SimBusLci>(o.geometry);
  auto * simp = sim.get();
  Chassis c(o, std::move(sim));
  ASSERT_TRUE(c.start());
  simp->set_remote_control(true);
  const auto end = Clock::now() + 1000ms;
  while (Clock::now() < end) {
    c.set_twist({1.0, 0.0, 0.0});
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_EQ(c.state().motion.source, ControlSource::kRemoteControl);
  EXPECT_NEAR(c.state().motion.twist.vx, 0.0, 0.05);
  c.stop();
}

// ---- Luchs B VCU upper interface (legacy B) -------------------------------------------------

TEST(LuchsBLegacy, CommandLayout)
{
  legacy_b::Config cfg;
  HostRequest req;
  LampCommand lamps;
  lamps.headlight = 1;
  const auto f = legacy_b::encode_command({30.0, 0.0}, 0.2113, 1.4, Gear::kDrive, req, lamps,
      0x42, cfg);
  EXPECT_EQ(f.id, 0x701u);
  // throttle 30 %, brake 0, -12 deg (wire + = right), cap 6 km/h, 200 deg/s, remote+headlight+D
  const std::array<uint8_t, 8> want{0x1E, 0x00, 0xF4, 0x06, 0xC8, 0x00, 0x13, 0x42};
  EXPECT_EQ(f.data, want);
  req.estop = true;
  const auto e = legacy_b::encode_command({}, 0.0, 0.0, Gear::kPark, req, {}, 0, cfg);
  EXPECT_EQ(e.data[6], 0x01 | 0x04 | 0x08);      // remote, e-stop, park, gear N
}

TEST(LuchsBLegacy, DecodeFeedback)
{
  legacy_b::Config cfg;
  ChassisState s;
  // SOC 80 %, 30 C, 75 V, fault level 0, gear R, remote mode, 7 km/h, heartbeat 9
  const CanFrame f{0x703, 8, {80, 70, 75, 0x00, 0x00, static_cast<uint8_t>((2 << 2) | 0x10), 7, 9}};
  ASSERT_TRUE(legacy_b::decode(f, cfg, s));
  EXPECT_DOUBLE_EQ(s.battery.soc_percent, 80.0);
  EXPECT_DOUBLE_EQ(s.battery.temperature_c, 30.0);
  EXPECT_DOUBLE_EQ(s.battery.voltage_v, 75.0);
  EXPECT_EQ(s.steer.gear, Gear::kReverse);
  EXPECT_EQ(s.motion.source, ControlSource::kHost);
  EXPECT_NEAR(s.motion.twist.vx, -7.0 / 3.6, 1e-9);
  EXPECT_FALSE(legacy_b::decode({0x704, 8, {}}, cfg, s));
}

TEST(LuchsBLegacy, SpeedControllerBrakesAtRestAndOnOverspeed)
{
  legacy_b::Config cfg;
  legacy_b::SpeedController sc;
  auto p = sc.step(0.0, 0.0, 0.01, cfg);
  EXPECT_DOUBLE_EQ(p.throttle, 0.0);
  EXPECT_DOUBLE_EQ(p.brake, cfg.hold_brake);
  p = sc.step(1.0, 0.0, 0.01, cfg);
  EXPECT_GT(p.throttle, 0.0);
  EXPECT_LE(p.throttle, cfg.max_throttle);
  p = sc.step(1.0, 2.5, 0.01, cfg);
  EXPECT_DOUBLE_EQ(p.throttle, 0.0);
  EXPECT_GT(p.brake, 0.0);
}

TEST(LuchsBSim, LegacyReachesSpeedAndStops)
{
  const auto o = ChassisOptions::for_build(Build::kLuchsB, Protocol::kLegacyB);
  auto sim = std::make_unique<SimBusLegacyB>(o.geometry, o.legacy_b);
  auto * simp = sim.get();
  Chassis c(o, std::move(sim));
  ASSERT_TRUE(c.start());
  auto end = Clock::now() + 4000ms;
  while (Clock::now() < end) {
    c.set_twist({1.5, 0.0, 0.0});
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_NEAR(simp->speed(), 1.5, 0.35);
  EXPECT_TRUE(c.status().feedback_ok);
  // reverse request while moving: brakes to a stop first, then reverses
  end = Clock::now() + 6000ms;
  while (Clock::now() < end) {
    c.set_twist({-0.8, 0.0, 0.0});
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_NEAR(simp->speed(), -0.8, 0.35);
  c.halt();
  std::this_thread::sleep_for(1500ms);
  EXPECT_NEAR(simp->speed(), 0.0, 0.1);
  c.stop();
}
