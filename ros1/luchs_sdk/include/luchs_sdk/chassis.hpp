// Copyright 2026 TerraAgent Robotics. Chassis: RJ Tech Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Thread-safe Luchs chassis client (Luchs A and Luchs B).
//
//   auto opt = luchs_sdk::ChassisOptions::for_build(luchs_sdk::Build::kLuchsB);  // or default: A-4WS
//   luchs_sdk::Chassis chassis(opt);
//   if (!chassis.start()) { std::cerr << chassis.last_error(); }
//   chassis.set_twist({0.3, 0.0, 0.2});            // call at >= 1/cmd_timeout Hz
//   auto s = chassis.state();                      // latest decoded feedback
//
// Safety behaviour (all in the SDK, independent of ROS):
//   - Command watchdog: no set_twist()/set_command() for `cmd_timeout_s` -> ramp to zero,
//     then gear P if `park_when_idle`.
//   - Feedback watchdog: no chassis feedback for `feedback_timeout_s` -> zero command,
//     status().feedback_ok = false.
//   - Acceleration limits on the commanded twist; steering and speed limits (Kinematics).
//   - stop() and the destructor send a zero command with gear P before closing the socket.
//   - LCI: the host heartbeat (0x5A5) goes out every cycle with a fresh AliveCounter, so the
//     chassis stops by itself within 200 ms if this process hangs or dies. The live limits the
//     chassis reports (0x5B0) are applied before the command is sent, so a clipped speed keeps
//     its path curvature.

#ifndef LUCHS_SDK__CHASSIS_HPP_
#define LUCHS_SDK__CHASSIS_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "luchs_sdk/kinematics.hpp"
#include "luchs_sdk/lci_protocol.hpp"
#include "luchs_sdk/legacy_a_protocol.hpp"
#include "luchs_sdk/legacy_b_protocol.hpp"
#include "luchs_sdk/socketcan.hpp"
#include "luchs_sdk/types.hpp"

namespace luchs_sdk
{

struct ChassisOptions
{
  std::string interface{"can0"};
  Geometry geometry{Geometry::for_build(Build::kFourWheelSteer)};
  std::optional<MotionModel> model;     ///< default: default_model(geometry.build)
  Protocol protocol_type{Protocol::kLegacyA};
  legacy_a::Config protocol;            ///< Luchs A protocol settings
  lci::Config lci;                      ///< LCI settings (reference_x is set from the geometry)
  legacy_b::Config legacy_b;            ///< Luchs B VCU upper-interface settings
  bool follow_live_limits{true};        ///< LCI: apply 0x5B0 limits before sending
  double tx_rate_hz{20.0};              ///< control cycle, 1..100 Hz (LCI: 50 Hz)
  double cmd_timeout_s{0.5};
  double feedback_timeout_s{0.5};
  double max_accel_x{1.5};              ///< m/s^2
  double max_accel_y{1.0};              ///< m/s^2
  double max_accel_wz{3.0};             ///< rad/s^2
  bool park_when_idle{true};            ///< gear P once the command has timed out and stopped
  bool send_host_time{true};            ///< 0x5A4, Unix time in ms (LCI: once a second)

  /// Defaults for a build: geometry, protocol (B: LCI) and control rate (B: 50 Hz).
  static ChassisOptions for_build(Build b, Protocol p);
  static ChassisOptions for_build(Build b);
};

/// CAN identifier of the frame that carries the measured body motion for a protocol
/// (0x5B5 for Luchs A and LCI, 0x703 for the Luchs B VCU upper interface).
uint32_t motion_feedback_id(const ChassisOptions & o);

class Chassis
{
public:
  struct Status
  {
    bool can_open{false};
    bool feedback_ok{false};
    bool command_active{false};
    bool limited{false};                ///< last request was reduced by a limit
    MotionModel model{MotionModel::kDifferential};
    Gear gear{Gear::kPark};
    double command_age_s{-1.0};
    double feedback_age_s{-1.0};
    uint64_t tx_cycles{0};
    uint64_t tx_errors{0};
    SteerSolution last;
    Protocol protocol{Protocol::kLegacyA};
    HostRequest host;                   ///< what the heartbeat currently requests
    uint8_t heartbeat_counter{0};
    bool live_limits_applied{false};    ///< LCI limits were fresh and used this cycle
    bool model_mismatch{false};         ///< LCI 0x5BF names a different chassis than configured
    legacy_b::Pedals pedals;            ///< legacy B: last pedal command
  };
  using FeedbackCallback = std::function<void(const ChassisState &, uint32_t can_id)>;

  explicit Chassis(ChassisOptions options);
  /// Uses an already-open transport (tests, other CAN back-ends). Takes ownership.
  Chassis(ChassisOptions options, std::unique_ptr<Transport> transport);
  ~Chassis();
  Chassis(const Chassis &) = delete;
  Chassis & operator=(const Chassis &) = delete;

  bool start();
  void stop();
  bool running() const {return running_;}

  void set_twist(const Twist2D & twist, Clock::time_point stamp = Clock::now());
  void set_command(const MotionCommand & command, Clock::time_point stamp = Clock::now());   ///< advanced: model, gear, explicit angles
  void set_model(MotionModel model);
  void set_lamps(const LampCommand & lamps);
  void halt();                                     ///< zero the command now (no ramp)

  /// Heartbeat requests (LCI, legacy B). estop is latched by the chassis until a maintenance
  /// reset; park is a controlled stop and hold.
  void set_host_request(const HostRequest & req);
  void request_estop(bool on);
  void request_park(bool on);
  void set_payload_power(int output, bool on);     ///< output 1 or 2
  void set_speed_limit(double mps);                ///< 0 = chassis default

  ChassisState state() const;
  Status status() const;
  const ChassisOptions & options() const {return opt_;}
  std::string last_error() const;
  void on_feedback(FeedbackCallback cb);

  /// One control cycle (normally run by the internal thread; public for tests).
  void tick(Clock::time_point now);
  /// Feeds one received frame (normally done by the internal receive thread). Use it when
  /// the application owns the CAN socket. Returns true if the frame was chassis feedback.
  bool process(const CanFrame & frame, Clock::time_point stamp = Clock::now());

private:
  void rx_loop();
  void tx_loop();
  static uint64_t unix_ms();
  Geometry effective_geometry(Clock::time_point now) const;   // with live limits (LCI)
  std::vector<CanFrame> encode_locked(const MotionCommand & out, const SteerSolution & sol,
    bool drive, Clock::time_point now, double dt);

  ChassisOptions opt_;
  std::unique_ptr<Transport> can_;
  mutable std::mutex mtx_;
  ChassisState state_;
  MotionCommand cmd_;
  bool cmd_explicit_{false};
  std::optional<Clock::time_point> cmd_stamp_;
  LampCommand lamps_;
  HostRequest host_;
  uint8_t counter_{0};
  std::optional<Clock::time_point> last_lamps_tx_, last_time_tx_;
  legacy_b::SpeedController speed_ctl_;
  double legacy_b_angle_{0.0};
  Gear legacy_b_gear_{Gear::kNeutral};
  Status status_;
  SlewLimiter slew_;
  std::optional<Clock::time_point> last_tick_;
  FeedbackCallback cb_;
  std::string error_;
  std::atomic<bool> running_{false};
  std::thread rx_, tx_;
};

}  // namespace luchs_sdk

#endif  // LUCHS_SDK__CHASSIS_HPP_
