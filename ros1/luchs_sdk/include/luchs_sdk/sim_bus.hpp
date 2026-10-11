// Copyright 2026 TerraAgent Robotics. Chassis: RJ Tech Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Software stand-ins for a Luchs chassis on the CAN bus. They consume the host's command frames
// and produce feedback frames with a first-order response, so the whole stack can be developed
// and tested without hardware:  luchs_driver dry_run:=true,  or  make_simulator(options).
// They model the protocol and the chassis' safety behaviour, not the vehicle dynamics.
//   SimBus         Luchs A protocol v1.1
//   SimBusLci      LCI v1.0 draft r3 as specified for Luchs B: heartbeat timeout, latched
//                  e-stop request, live limits with clip flags, IMU, version 0x1B
//   SimBusLegacyB  Luchs B VCU upper interface V2.6 (pedal-level 0x701 / 0x703)

#ifndef LUCHS_SDK__SIM_BUS_HPP_
#define LUCHS_SDK__SIM_BUS_HPP_

#include <deque>
#include <mutex>
#include <string>

#include "luchs_sdk/chassis.hpp"
#include "luchs_sdk/kinematics.hpp"
#include "luchs_sdk/legacy_a_protocol.hpp"
#include "luchs_sdk/socketcan.hpp"

namespace luchs_sdk
{

class SimBus : public Transport
{
public:
  explicit SimBus(Geometry g = Geometry::for_build(Build::kFourWheelSteer),
    legacy_a::Config cfg = {}, double time_constant_s = 0.25);

  bool write(const CanFrame & frame) override;
  std::optional<CanFrame> read(int timeout_ms) override;
  bool is_open() const override {return true;}
  const std::string & last_error() const override {return err_;}

  void press_estop(bool pressed);   ///< test hook: protection frame reports e-stop

private:
  void step(Clock::time_point now);

  Geometry g_;
  legacy_a::Config cfg_;
  double tau_;
  std::mutex mtx_;
  std::deque<CanFrame> rx_;
  Twist2D target_{}, actual_{};
  MotionModel model_{MotionModel::kFourWheelSteer};
  uint8_t gear_{1};
  double front_{0.0}, rear_{0.0};
  LampCommand lamps_{};
  bool estop_{false};
  double soc_{87.5};
  std::optional<Clock::time_point> last_;
  Clock::time_point next_emit_{};
  std::string err_;
};

class SimBusLci : public Transport
{
public:
  explicit SimBusLci(Geometry g = Geometry::for_build(Build::kLuchsB), double time_constant_s = 0.3);

  bool write(const CanFrame & frame) override;
  std::optional<CanFrame> read(int timeout_ms) override;
  bool is_open() const override {return true;}
  const std::string & last_error() const override {return err_;}

  // test hooks
  void press_estop(bool pressed);        ///< onboard button: stop while pressed
  void set_remote_control(bool rc);      ///< transmitter takes over (host commands ignored)
  void maintenance_reset();              ///< clears a latched host e-stop request
  void set_terrain_speed_limit(double mps);   ///< e.g. a slope derate; 0 = none

private:
  void step(Clock::time_point now);
  void emit(Clock::time_point now);

  Geometry g_;
  double tau_;
  std::mutex mtx_;
  std::deque<CanFrame> rx_;
  // received
  Twist2D target_{};                     // at the reference point
  MotionModel model_{MotionModel::kDualAckermann};
  uint8_t gear_{1};
  double cmd_front_{0.0}, cmd_rear_{0.0};
  uint8_t steer_source_{0};
  LampCommand lamps_{};
  HostRequest req_{};
  std::optional<Clock::time_point> last_hb_;
  std::optional<uint8_t> last_counter_;
  Clock::time_point counter_changed_{};
  // state
  double vx_{0.0}, front_{0.0}, rear_{0.0}, wz_{0.0}, odom_yaw_{0.0};
  bool estop_button_{false}, estop_latched_{false}, rc_{false}, hb_timeout_{false};
  double terrain_limit_{0.0};
  bool speed_clipped_{false}, steer_clipped_{false};
  uint8_t alive_{0}, imu_counter_{0};
  uint32_t ticks_{0};
  double soc_{86.0};
  std::optional<Clock::time_point> last_;
  Clock::time_point next_emit_{};
  std::string err_;
};

class SimBusLegacyB : public Transport
{
public:
  explicit SimBusLegacyB(Geometry g = Geometry::for_build(Build::kLuchsB),
    legacy_b::Config cfg = {});

  bool write(const CanFrame & frame) override;
  std::optional<CanFrame> read(int timeout_ms) override;
  bool is_open() const override {return true;}
  const std::string & last_error() const override {return err_;}

  double speed() const;                  ///< true simulated speed, m/s (signed)

private:
  Geometry g_;
  legacy_b::Config cfg_;
  mutable std::mutex mtx_;
  std::deque<CanFrame> rx_;
  double throttle_{0.0}, brake_{0.0}, cap_kmh_{0.0};
  uint8_t gear_{0};
  bool park_{true}, estop_{false}, remote_{false};
  std::optional<Clock::time_point> last_cmd_;
  double v_{0.0};                        // >= 0, direction from the gear
  uint8_t hb_{0};
  std::optional<Clock::time_point> last_;
  Clock::time_point next_emit_{};
  std::string err_;
};

/// The simulator that matches `options.protocol_type` and the geometry.
std::unique_ptr<Transport> make_simulator(const ChassisOptions & options);

}  // namespace luchs_sdk

#endif  // LUCHS_SDK__SIM_BUS_HPP_
