// Copyright 2026 TerraAgent Robotics. Chassis: RJ Tech Limited.
// SPDX-License-Identifier: Apache-2.0

#include "luchs_driver/luchs_driver_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "luchs_sdk/sim_bus.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace luchs_driver
{
using luchs_sdk::Build;
using luchs_sdk::MotionModel;
namespace la = luchs_sdk::legacy_a;

namespace
{
const char * kWheelNames[4] = {"front_left", "front_right", "rear_left", "rear_right"};

template<typename T>
T param(rclcpp::Node & n, const std::string & name, const T & def, const std::string & doc)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = doc;
  return n.declare_parameter<T>(name, def, d);
}

bool steers(Build b, std::size_t wheel)
{
  switch (b) {
    case Build::kSkidSteer: return false;
    case Build::kFrontAckermann: return wheel < 2;
    default: return true;   // A-DA, A-4WS, B (front and rear EPS)
  }
}
double age(const std::optional<luchs_sdk::Clock::time_point> & t)
{
  return t ? std::chrono::duration<double>(luchs_sdk::Clock::now() - *t).count() : 1e9;
}
}  // namespace

Build parse_build(const std::string & s)
{
  if (s == "df") {return Build::kSkidSteer;}
  if (s == "fa") {return Build::kFrontAckermann;}
  if (s == "da") {return Build::kDualAckermann;}
  if (s == "4ws") {return Build::kFourWheelSteer;}
  if (s == "b") {return Build::kLuchsB;}
  throw std::invalid_argument("build must be one of df, fa, da, 4ws (Luchs A) or b (Luchs B) "
          "(got '" + s + "')");
}

luchs_sdk::Protocol parse_protocol(const std::string & s, Build b)
{
  if (s.empty()) {return b == Build::kLuchsB ? luchs_sdk::Protocol::kLci : luchs_sdk::Protocol::kLegacyA;}
  if (s == "a") {return luchs_sdk::Protocol::kLegacyA;}
  if (s == "lci") {return luchs_sdk::Protocol::kLci;}
  if (s == "legacy_b") {return luchs_sdk::Protocol::kLegacyB;}
  throw std::invalid_argument("protocol.type must be a, lci or legacy_b (got '" + s + "')");
}

MotionModel parse_model(const std::string & s)
{
  if (s == "diff") {return MotionModel::kDifferential;}
  if (s == "ackermann") {return MotionModel::kFrontAckermann;}
  if (s == "dual_ackermann") {return MotionModel::kDualAckermann;}
  if (s == "4ws") {return MotionModel::kFourWheelSteer;}
  throw std::invalid_argument(
          "motion_model must be one of diff, ackermann, dual_ackermann, 4ws (got '" + s + "')");
}

la::WheelOrder parse_order(const std::string & s)
{
  if (s == "lf_rf_lr_rr") {return la::WheelOrder::kLfRfLrRr;}
  if (s == "lf_lr_rf_rr") {return la::WheelOrder::kLfLrRfRr;}
  throw std::invalid_argument("wheel order must be lf_rf_lr_rr or lf_lr_rf_rr (got '" + s + "')");
}

luchs_sdk::ChassisOptions declare_options(rclcpp::Node & n)
{
  const Build b = parse_build(param<std::string>(n, "build", "4ws",
      "Chassis: Luchs A df (skid-steer), fa (front Ackermann), da (dual Ackermann), 4ws; "
      "or b (Luchs B)"));
  const auto proto = parse_protocol(param<std::string>(n, "protocol.type", "",
      "CAN protocol: a (Luchs A v1.1), lci (LCI v1.0), legacy_b (Luchs B VCU V2.6). "
      "Empty = default for the build (A: a, B: lci)"), b);
  luchs_sdk::ChassisOptions o = luchs_sdk::ChassisOptions::for_build(b, proto);
  o.interface = param<std::string>(n, "can_interface", "can0",
      "SocketCAN interface (Luchs A and LCI: 1 Mbit/s)");
  const std::string model = param<std::string>(n, "motion_model", "",
      "Model for cmd_vel: diff, ackermann, dual_ackermann, 4ws. Empty = default for the build");
  if (!model.empty()) {o.model = parse_model(model);}

  auto & g = o.geometry;
  g.wheelbase = param(n, "geometry.wheelbase", g.wheelbase, "m");
  g.track = param(n, "geometry.track", g.track, "m");
  g.wheel_radius = param(n, "geometry.wheel_radius", g.wheel_radius, "m");
  g.max_steer_rad = param(n, "geometry.max_steer_angle", g.max_steer_rad, "rad, per wheel");
  g.max_vx = param(n, "limits.max_vx", g.max_vx,
      "m/s (vehicle maximum: Luchs A 3.2 m/s, Luchs B 11.1 m/s)");
  g.max_vy = param(n, "limits.max_vy", g.max_vy, "m/s, four-wheel-steer model only");
  g.max_wz = param(n, "limits.max_wz", g.max_wz, "rad/s");
  g.allow_pivot_turn = param(n, "limits.allow_pivot_turn", g.allow_pivot_turn,
      "Ackermann builds: turn on the spot (differential model) for a pure yaw request");
  o.max_accel_x = param(n, "limits.max_accel_x", o.max_accel_x, "m/s^2");
  o.max_accel_y = param(n, "limits.max_accel_y", o.max_accel_y, "m/s^2");
  o.max_accel_wz = param(n, "limits.max_accel_wz", o.max_accel_wz, "rad/s^2");

  o.tx_rate_hz = param(n, "control.rate", o.tx_rate_hz,
      "Hz, 1..100. Luchs A 20 Hz; LCI 50 Hz (heartbeat every cycle); Luchs B VCU 100 Hz");
  o.cmd_timeout_s = param(n, "control.cmd_timeout", o.cmd_timeout_s,
      "s without a command before the chassis is stopped");
  o.feedback_timeout_s = param(n, "control.feedback_timeout", o.feedback_timeout_s,
      "s without chassis feedback before commands are suppressed");
  o.park_when_idle = param(n, "control.park_when_idle", o.park_when_idle,
      "Engage gear P (motor brake) when stopped and idle");
  o.send_host_time = param(n, "control.send_host_time", o.send_host_time,
      "Send 0x5A4 host time (Unix ms): every cycle (Luchs A), once a second (LCI)");

  // LCI (Luchs B)
  o.follow_live_limits = param(n, "lci.follow_live_limits", o.follow_live_limits,
      "Apply the chassis' live limits (0x5B0) before sending, keeping path curvature");
  o.lci.accel_limit = param(n, "lci.accel_limit", o.lci.accel_limit,
      "m/s^2 sent in 0x5A1 AccelLimit; 0 = chassis default ramp");
  o.lci.check_checksum = param(n, "lci.check_checksum", o.lci.check_checksum,
      "Ignore 0x5BD frames with a wrong checksum");

  // Luchs B VCU upper interface V2.6 (units shipping before LCI firmware)
  auto & lb = o.legacy_b;
  lb.cmd_id = static_cast<uint32_t>(param<int>(n, "legacy_b.cmd_id", static_cast<int>(lb.cmd_id),
      "UpperCmd identifier, default 0x701 (1793). TO CONFIRM: the sheet writes 701"));
  lb.fb_id = static_cast<uint32_t>(param<int>(n, "legacy_b.fb_id", static_cast<int>(lb.fb_id),
      "VCU2Upper identifier, default 0x703 (1795)"));
  lb.steer_sign = param<int>(n, "legacy_b.steer_sign", lb.steer_sign,
      "-1: wire angle positive = steer right (EPS convention). TO CONFIRM with wheels lifted");
  if (lb.steer_sign != 1 && lb.steer_sign != -1) {
    throw std::invalid_argument("legacy_b.steer_sign must be 1 or -1");
  }
  lb.max_wheel_angle_deg = param(n, "legacy_b.max_wheel_angle_deg", lb.max_wheel_angle_deg, "deg");
  lb.steer_rate_dps = param(n, "legacy_b.steer_rate", lb.steer_rate_dps, "deg/s, 50..520");
  lb.speed_cap_kmh = param(n, "legacy_b.speed_cap_kmh", lb.speed_cap_kmh, "km/h, 0..30");
  lb.throttle_ff = param(n, "legacy_b.throttle_ff", lb.throttle_ff, "% per m/s");
  lb.throttle_kp = param(n, "legacy_b.throttle_kp", lb.throttle_kp, "% per m/s");
  lb.throttle_ki = param(n, "legacy_b.throttle_ki", lb.throttle_ki, "% per m");
  lb.max_throttle = param(n, "legacy_b.max_throttle", lb.max_throttle, "%");
  lb.brake_kp = param(n, "legacy_b.brake_kp", lb.brake_kp, "% per m/s overspeed");
  lb.hold_brake = param(n, "legacy_b.hold_brake", lb.hold_brake, "% while stopped");
  lb.max_brake = param(n, "legacy_b.max_brake", lb.max_brake, "%");

  auto & p = o.protocol;
  p.wheel_steer_cmd_id = static_cast<uint32_t>(param<int>(n, "protocol.wheel_steer_cmd_id",
      static_cast<int>(p.wheel_steer_cmd_id), "0x5A2 (1442) or 0x5A3 (1443); lamps use the other"));
  if (p.wheel_steer_cmd_id != la::kId5A2 && p.wheel_steer_cmd_id != la::kId5A3) {
    throw std::invalid_argument("protocol.wheel_steer_cmd_id must be 1442 (0x5A2) or 1443 (0x5A3)");
  }
  p.wheel_steer_cmd_order = parse_order(param<std::string>(n, "protocol.wheel_steer_cmd_order",
      "lf_rf_lr_rr", "Wheel order in the wheel-angle command"));
  p.wheel_steer_fb_order = parse_order(param<std::string>(n, "protocol.wheel_steer_fb_order",
      "lf_lr_rf_rr", "Wheel order in 0x5B7"));
  p.wheel_speed_fb_order = parse_order(param<std::string>(n, "protocol.wheel_speed_fb_order",
      "lf_lr_rf_rr", "Wheel order in 0x5B9"));
  p.wheel_speed_scale = param(n, "protocol.wheel_speed_scale", p.wheel_speed_scale,
      "0x5B9 raw -> rad/s");
  p.ultrasonic_scale_m = param(n, "protocol.ultrasonic_scale", p.ultrasonic_scale_m,
      "0x5B8 raw -> m");
  p.battery_soc_scale = param(n, "protocol.battery_soc_scale", p.battery_soc_scale,
      "0x5BB raw -> %");
  p.battery_temp_scale = param(n, "protocol.battery_temperature_scale", p.battery_temp_scale,
      "0x5BB raw -> deg C");
  p.battery_voltage_scale = param(n, "protocol.battery_voltage_scale", p.battery_voltage_scale,
      "0x5BB raw -> V");
  p.charging_scale = param(n, "protocol.charging_scale", p.charging_scale, "0x5BB raw -> %");
  return o;
}

LuchsDriverNode::LuchsDriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("luchs_driver", options)
{
  opt_ = declare_options(*this);
  dry_run_ = param(*this, "dry_run", false,
      "Run against the built-in chassis simulator instead of CAN (no hardware needed)");
  odom_frame_ = param<std::string>(*this, "odom.frame_id", "odom", "");
  base_frame_ = param<std::string>(*this, "odom.child_frame_id", "base_footprint", "");
  publish_tf_ = param(*this, "odom.publish_tf", true,
      "Broadcast odom -> base TF. Set false when an EKF publishes it");
  joint_prefix_ = param<std::string>(*this, "joint_prefix", "", "Prefix for joint names");
  battery_low_percent_ = param(*this, "battery_low_percent", 15.0, "Diagnostics warning level");
  const double rate = param(*this, "publish_rate", 50.0, "Hz for state, odometry and joints");
  const bool stamped = param(*this, "cmd_vel_stamped", false,
      "Subscribe to geometry_msgs/TwistStamped instead of Twist on cmd_vel");

  imu_frame_ = param<std::string>(*this, "imu.frame_id", "base_link",
      "Frame of the chassis IMU (LCI 0x5BE / 0x5B1); vehicle frame by definition");
  battery_capacity_ah_ = param(*this, "battery.design_capacity",
      opt_.geometry.build == Build::kLuchsB ? 60.0 : 15.0,
      "Ah. Luchs A 15 Ah (48 V); Luchs B 60 Ah per 72 V pack, set to match the fitted packs");
  const double speed_limit = param(*this, "lci.speed_limit", 0.0,
      "m/s, host speed cap sent in the heartbeat; 0 = chassis default");

  chassis_ = dry_run_ ?
    std::make_unique<luchs_sdk::Chassis>(opt_, luchs_sdk::make_simulator(opt_)) :
    std::make_unique<luchs_sdk::Chassis>(opt_);
  chassis_->set_speed_limit(speed_limit);
  chassis_->on_feedback([this](const luchs_sdk::ChassisState & s, uint32_t id) {
      on_feedback(s, id);
    });

  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  pub_state_ = create_publisher<luchs_msgs::msg::ChassisState>("~/state", 10);
  pub_safety_ = create_publisher<luchs_msgs::msg::SafetyState>("~/safety", 10);
  pub_battery_ = create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
  pub_joints_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  if (opt_.protocol_type == luchs_sdk::Protocol::kLci) {
    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
  }
  if (publish_tf_) {tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);}

  if (stamped) {
    sub_cmd_vel_stamped_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10, [this](const geometry_msgs::msg::TwistStamped & m) {on_cmd_vel(m.twist);});
  } else {
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](const geometry_msgs::msg::Twist & m) {on_cmd_vel(m);});
  }
  sub_drive_ = create_subscription<luchs_msgs::msg::DriveCommand>(
    "~/drive_command", 10, [this](const luchs_msgs::msg::DriveCommand & m) {on_drive_command(m);});
  sub_lamps_ = create_subscription<luchs_msgs::msg::LampCommand>(
    "~/lamps", 10, [this](const luchs_msgs::msg::LampCommand & m) {on_lamps(m);});

  srv_reset_odom_ = create_service<std_srvs::srv::Trigger>("~/reset_odometry",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        std::lock_guard<std::mutex> lk(odom_mtx_);
        odom_.reset();
        res->success = true;
        res->message = "odometry reset to zero";
      });
  srv_model_ = create_service<luchs_msgs::srv::SetMotionModel>("~/set_motion_model",
      [this](const std::shared_ptr<luchs_msgs::srv::SetMotionModel::Request> req,
      std::shared_ptr<luchs_msgs::srv::SetMotionModel::Response> res) {
        const auto m = static_cast<MotionModel>(req->motion_model);
        if (req->motion_model < 1 || req->motion_model > 4 ||
        !luchs_sdk::model_supported(opt_.geometry.build, m))
        {
          res->success = false;
          res->message = "motion model not supported by this build";
          return;
        }
        chassis_->set_model(m);
        res->success = true;
        res->message = "motion model set";
      });

  // Heartbeat requests: LCI and the Luchs B VCU interface. Luchs A has no such fields.
  const bool has_heartbeat = opt_.protocol_type != luchs_sdk::Protocol::kLegacyA;
  const bool is_lci = opt_.protocol_type == luchs_sdk::Protocol::kLci;
  srv_estop_ = create_service<std_srvs::srv::SetBool>("~/estop",
      [this, has_heartbeat](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        if (!has_heartbeat) {
          res->success = false;
          res->message = "not available on the Luchs A protocol: use the emergency stop button";
          return;
        }
        chassis_->request_estop(req->data);
        res->success = true;
        res->message = req->data ?
        "emergency stop requested (latched by the chassis until a maintenance reset)" :
        "emergency stop request cleared";
      });
  srv_park_ = create_service<std_srvs::srv::SetBool>("~/park",
      [this, has_heartbeat](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        if (!has_heartbeat) {
          res->success = false;
          res->message = "not available on the Luchs A protocol (gear P is engaged when idle)";
          return;
        }
        chassis_->request_park(req->data);
        res->success = true;
        res->message = req->data ? "park requested" : "park request cleared";
      });
  srv_payload_ = create_service<luchs_msgs::srv::SetPayloadPower>("~/set_payload_power",
      [this, is_lci](const std::shared_ptr<luchs_msgs::srv::SetPayloadPower::Request> req,
      std::shared_ptr<luchs_msgs::srv::SetPayloadPower::Response> res) {
        if (!is_lci || (req->output != 1 && req->output != 2)) {
          res->success = false;
          res->message = is_lci ? "output must be 1 or 2" : "LCI only";
          return;
        }
        chassis_->set_payload_power(req->output, req->on);
        res->success = true;
        res->message = "payload power request sent";
      });
  srv_speed_limit_ = create_service<luchs_msgs::srv::SetSpeedLimit>("~/set_speed_limit",
      [this, is_lci](const std::shared_ptr<luchs_msgs::srv::SetSpeedLimit::Request> req,
      std::shared_ptr<luchs_msgs::srv::SetSpeedLimit::Response> res) {
        if (!is_lci || !(req->speed_limit >= 0.0)) {
          res->success = false;
          res->message = is_lci ? "speed_limit must be >= 0" : "LCI only (use limits.max_vx)";
          return;
        }
        chassis_->set_speed_limit(req->speed_limit);
        res->success = true;
        res->message = "speed limit sent in the heartbeat";
      });

  diag_ = std::make_unique<diagnostic_updater::Updater>(this);
  diag_->setHardwareID("luchs:" + opt_.interface);
  diag_->add("chassis", this, &LuchsDriverNode::diagnose);

  if (!chassis_->start()) {
    RCLCPP_FATAL(get_logger(), "cannot open CAN interface '%s': %s", opt_.interface.c_str(),
      chassis_->last_error().c_str());
    throw std::runtime_error(chassis_->last_error());
  }
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::clamp(rate, 1.0, 200.0)),
      [this]() {publish();});
  static const char * kModelNames[] = {"?", "differential", "front Ackermann", "four-wheel steer",
    "dual Ackermann"};
  RCLCPP_INFO(get_logger(), "Luchs driver %s on %s: Luchs %s, %s, %s, %.0f Hz, cmd timeout %.2f s%s",
    luchs_sdk::kSdkVersion, opt_.interface.c_str(), luchs_sdk::build_name(opt_.geometry.build),
    luchs_sdk::protocol_name(opt_.protocol_type),
    kModelNames[static_cast<int>(chassis_->status().model)],
    opt_.tx_rate_hz, opt_.cmd_timeout_s, dry_run_ ? " (DRY RUN: built-in simulator, no CAN)" : "");
  if (opt_.protocol_type == luchs_sdk::Protocol::kLegacyB) {
    RCLCPP_WARN(get_logger(), "Luchs B VCU upper interface: pedal-level control with 1 km/h speed "
      "feedback and no steering feedback. For bring-up and teleoperation; use LCI for autonomy.");
  }
}

LuchsDriverNode::~LuchsDriverNode()
{
  if (chassis_) {chassis_->stop();}   // sends zero speed + gear P before closing
}

void LuchsDriverNode::on_cmd_vel(const geometry_msgs::msg::Twist & m)
{
  chassis_->set_twist({m.linear.x, m.linear.y, m.angular.z});
}

void LuchsDriverNode::on_drive_command(const luchs_msgs::msg::DriveCommand & m)
{
  luchs_sdk::MotionCommand c;
  c.twist = {m.twist.linear.x, m.twist.linear.y, m.twist.angular.z};
  c.model = m.motion_model == 0 ? chassis_->status().model :
    static_cast<MotionModel>(m.motion_model);
  if (m.use_explicit_angles) {
    c.front_steer_rad = m.front_steer_angle;
    c.rear_steer_rad = m.rear_steer_angle;
    c.wheel_steer_rad = luchs_sdk::PerWheel{m.wheel_steer_angle[0], m.wheel_steer_angle[1],
      m.wheel_steer_angle[2], m.wheel_steer_angle[3]};
  }
  chassis_->set_command(c);
}

void LuchsDriverNode::on_lamps(const luchs_msgs::msg::LampCommand & m)
{
  luchs_sdk::LampCommand l;
  const uint8_t max_turn = opt_.protocol_type == luchs_sdk::Protocol::kLci ? 3 : 2;
  l.turn = static_cast<luchs_sdk::TurnSignal>(std::min<uint8_t>(m.turn_signal, max_turn));
  l.reverse = m.reverse_lamp;
  l.fault = m.fault_lamp;
  l.headlight = std::min<uint8_t>(m.headlight, 2);
  l.horn = m.horn;
  l.beacon = m.beacon;
  chassis_->set_lamps(l);
}

void LuchsDriverNode::on_feedback(const luchs_sdk::ChassisState & s, uint32_t id)
{
  if (id != luchs_sdk::motion_feedback_id(opt_) || !s.motion_stamp) {return;}
  std::lock_guard<std::mutex> lk(odom_mtx_);
  if (last_motion_) {
    const double dt = std::chrono::duration<double>(*s.motion_stamp - *last_motion_).count();
    odom_.update(s.motion.twist, dt);
  }
  last_motion_ = s.motion_stamp;
}

void LuchsDriverNode::publish()
{
  const auto s = chassis_->state();
  const auto st = chassis_->status();
  const auto now = this->now();
  const double dt = last_publish_ ? (now - *last_publish_).seconds() : 0.0;
  last_publish_ = now;

  // ---- odometry + TF -----------------------------------------------------------------
  double x, y, yaw;
  {
    std::lock_guard<std::mutex> lk(odom_mtx_);
    x = odom_.x();
    y = odom_.y();
    yaw = odom_.yaw();
  }
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  nav_msgs::msg::Odometry od;
  od.header.stamp = now;
  od.header.frame_id = odom_frame_;
  od.child_frame_id = base_frame_;
  od.pose.pose.position.x = x;
  od.pose.pose.position.y = y;
  od.pose.pose.orientation.x = q.x();
  od.pose.pose.orientation.y = q.y();
  od.pose.pose.orientation.z = q.z();
  od.pose.pose.orientation.w = q.w();
  od.twist.twist.linear.x = s.motion.twist.vx;
  od.twist.twist.linear.y = s.motion.twist.vy;
  od.twist.twist.angular.z = s.motion.twist.wz;
  // Planar wheel odometry: x, y, yaw observed; z, roll, pitch unobserved (large variance).
  const bool still = std::abs(s.motion.twist.vx) < 1e-3 && std::abs(s.motion.twist.vy) < 1e-3 &&
    std::abs(s.motion.twist.wz) < 1e-3;
  const double pv = still ? 1e-6 : 1e-2, yv = still ? 1e-6 : 3e-2, big = 1e6;
  od.pose.covariance = {pv, 0, 0, 0, 0, 0, 0, pv, 0, 0, 0, 0, 0, 0, big, 0, 0, 0,
    0, 0, 0, big, 0, 0, 0, 0, 0, 0, big, 0, 0, 0, 0, 0, 0, yv};
  od.twist.covariance = {1e-3, 0, 0, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 0, 0, big, 0, 0, 0,
    0, 0, 0, big, 0, 0, 0, 0, 0, 0, big, 0, 0, 0, 0, 0, 0, 2e-3};
  pub_odom_->publish(od);
  if (tf_) {
    geometry_msgs::msg::TransformStamped t;
    t.header = od.header;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.rotation = od.pose.pose.orientation;
    tf_->sendTransform(t);
  }

  // ---- chassis state -------------------------------------------------------------------
  luchs_msgs::msg::ChassisState cs;
  cs.header.stamp = now;
  cs.header.frame_id = base_frame_;
  cs.control_source = static_cast<uint8_t>(s.motion.source);
  cs.motion_model = s.motion.model_raw;
  cs.gear = s.steer.gear_raw;
  cs.twist = od.twist.twist;
  cs.front_steer_angle = s.steer.front_rad;
  cs.rear_steer_angle = s.steer.rear_rad;
  for (std::size_t i = 0; i < 4; ++i) {
    cs.wheel_steer_angle[i] = s.wheel_steer_rad[i];
    cs.wheel_speed[i] = s.wheel_speed[i];
  }
  cs.fault_code = s.safety.fault_code;
  cs.fault_text = luchs_sdk::fault_code_text(s.safety.fault_code);
  cs.motor_fault_high = s.motor_fault_high;
  cs.motor_fault_low = s.motor_fault_low;
  cs.turn_signal = s.lamps.turn_raw;
  cs.reverse_lamp = s.lamps.reverse_raw != 0;
  cs.fault_lamp = s.lamps.fault_raw != 0;
  cs.light_rain_raw = s.lamps.light_rain_raw;
  cs.feedback_ok = st.feedback_ok;
  cs.command_active = st.command_active;
  cs.command_limited = st.limited;
  cs.commanded_motion_model = static_cast<uint8_t>(st.model);
  cs.commanded_twist.linear.x = st.last.twist.vx;
  cs.commanded_twist.linear.y = st.last.twist.vy;
  cs.commanded_twist.angular.z = st.last.twist.wz;
  cs.heartbeat_counter = st.heartbeat_counter;
  cs.estop_requested = st.host.estop;
  cs.park_requested = st.host.park;
  cs.payload_power_1 = st.host.payload_power_1;
  cs.payload_power_2 = st.host.payload_power_2;
  cs.host_speed_limit = st.host.speed_limit;
  cs.model_mismatch = st.model_mismatch;
  // LCI block
  cs.lci_ok = opt_.protocol_type == luchs_sdk::Protocol::kLci && age(s.status_stamp) < 0.5;
  cs.operating_mode = s.status.operating_mode;
  cs.estop_source_bits = s.status.estop_source_bits;
  cs.estop_source_text = luchs_sdk::estop_source_text(s.status.estop_source_bits);
  cs.fault_level = s.status.fault_level;
  cs.rc_online = s.status.rc_online;
  cs.rc_enabled = s.status.rc_enabled;
  cs.rc_link = s.status.rc_link;
  cs.four_wheel_drive = s.status.four_wd;
  cs.park_brake_engaged = s.status.park_brake;
  cs.active_speed_limit = s.status.active_speed_limit;
  cs.max_vx_forward = s.limits.max_vx_fwd;
  cs.max_vx_reverse = s.limits.max_vx_rev;
  cs.max_steer_angle = s.limits.max_steer_rad;
  cs.limit_reason_bits = s.limits.reason_bits;
  cs.limit_reason_text = luchs_sdk::limit_reason_text(s.limits.reason_bits);
  cs.speed_clipped = s.limits.speed_clipped;
  cs.steer_clipped = s.limits.steer_clipped;
  cs.brake_pressure = s.brake.brake_pressure_mpa;
  cs.epb_state = s.brake.epb_state;
  cs.eps_front_state = s.brake.eps_front;
  cs.eps_rear_state = s.brake.eps_rear;
  cs.brake_state = s.brake.brake_state;
  for (std::size_t i = 0; i < 4; ++i) {
    cs.motor_temperature[i] = s.temperature.motor[i];
    cs.controller_temperature[i] = s.temperature.controller[i];
  }
  if (s.version_stamp) {
    cs.model_id = s.version.model_id;
    cs.model_text = luchs_sdk::model_id_text(s.version.model_id);
    cs.firmware_version = std::to_string(s.version.fw_major) + "." +
      std::to_string(s.version.fw_minor) + "." + std::to_string(s.version.fw_patch);
    cs.lci_version = std::to_string(s.version.lci_major) + "." + std::to_string(s.version.lci_minor);
  }
  if (opt_.protocol_type == luchs_sdk::Protocol::kLegacyB) {
    cs.fault_text = luchs_sdk::legacy_b::fault_text(s.status.fault_level);
  }
  pub_state_->publish(cs);

  // ---- chassis IMU (LCI) ---------------------------------------------------------------
  if (pub_imu_ && age(s.imu_rate_stamp) < 0.1) {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now;
    imu.header.frame_id = joint_prefix_ + imu_frame_;
    imu.orientation_covariance[0] = -1.0;     // no orientation estimate (roll / pitch in ~/state)
    imu.angular_velocity.x = s.imu.rate[0];
    imu.angular_velocity.y = s.imu.rate[1];
    imu.angular_velocity.z = s.imu.rate[2];
    imu.linear_acceleration.x = s.imu.accel[0];
    imu.linear_acceleration.y = s.imu.accel[1];
    imu.linear_acceleration.z = s.imu.accel[2];
    imu.angular_velocity_covariance = {1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4};        // TO CONFIRM
    imu.linear_acceleration_covariance = {1e-2, 0, 0, 0, 1e-2, 0, 0, 0, 1e-2};
    pub_imu_->publish(imu);
  }

  luchs_msgs::msg::SafetyState sf;
  sf.header = cs.header;
  sf.estop_left = s.safety.estop_raw == 1;
  sf.estop_right = s.safety.estop_raw == 2;
  sf.bumper_front = s.safety.bumper_raw == 1;
  sf.bumper_rear = s.safety.bumper_raw == 2;
  sf.ultrasonic_trigger = s.safety.ultrasonic_trigger_raw;
  for (std::size_t i = 0; i < 4; ++i) {
    sf.ultrasonic_range[i] = static_cast<float>(s.safety.ultrasonic_m[i]);
  }
  sf.fault_code = cs.fault_code;
  sf.fault_text = cs.fault_text;
  pub_safety_->publish(sf);

  // ---- battery -------------------------------------------------------------------------
  if (s.battery_stamp) {
    sensor_msgs::msg::BatteryState b;
    b.header = cs.header;
    b.voltage = static_cast<float>(s.battery.voltage_v);
    b.temperature = static_cast<float>(s.battery.temperature_c);
    b.current = std::numeric_limits<float>::quiet_NaN();
    b.charge = std::numeric_limits<float>::quiet_NaN();
    b.capacity = std::numeric_limits<float>::quiet_NaN();
    b.design_capacity = static_cast<float>(battery_capacity_ah_);
    b.percentage = static_cast<float>(std::clamp(s.battery.soc_percent / 100.0, 0.0, 1.0));
    b.power_supply_status = (opt_.protocol_type == luchs_sdk::Protocol::kLegacyA ?
      s.safety.fault_code == 0x04 : s.battery.charging_percent > 0.0) ?
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING :
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    b.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE;
    b.present = true;
    pub_battery_->publish(b);
  }

  // ---- joint states (for robot_state_publisher) ----------------------------------------
  const Build build = opt_.geometry.build;
  luchs_sdk::PerWheel steer{}, spin = s.wheel_speed;
  if (build == Build::kFourWheelSteer) {
    steer = s.wheel_steer_rad;
  } else if (build == Build::kLuchsB && s.steer_stamp) {
    steer = luchs_sdk::axle_to_wheel_angles(s.steer.front_rad, s.steer.rear_rad, opt_.geometry);
  } else if (build != Build::kSkidSteer) {
    const auto model = s.motion.model_raw >= 1 && s.motion.model_raw <= 4 ?
      static_cast<MotionModel>(s.motion.model_raw) : luchs_sdk::default_model(build);
    steer = luchs_sdk::solve(s.motion.twist, model, opt_.geometry).wheel_rad;
  }
  if (!s.wheel_speed_stamp) {   // no wheel-speed frame: derive from the body twist
    const auto sol = luchs_sdk::solve(s.motion.twist, st.model, opt_.geometry);
    for (std::size_t i = 0; i < 4; ++i) {spin[i] = sol.wheel_speed[i] / opt_.geometry.wheel_radius;}
  }
  sensor_msgs::msg::JointState js;
  js.header.stamp = now;
  for (std::size_t i = 0; i < 4; ++i) {
    if (steers(build, i)) {
      js.name.push_back(joint_prefix_ + kWheelNames[i] + "_steer_joint");
      js.position.push_back(steer[i]);
      js.velocity.push_back(0.0);
    }
  }
  for (std::size_t i = 0; i < 4; ++i) {
    if (st.feedback_ok && dt > 0.0 && dt < 1.0) {wheel_pos_[i] += spin[i] * dt;}
    js.name.push_back(joint_prefix_ + kWheelNames[i] + "_wheel_joint");
    js.position.push_back(std::remainder(wheel_pos_[i], 2.0 * M_PI));
    js.velocity.push_back(spin[i]);
  }
  pub_joints_->publish(js);
}

void LuchsDriverNode::diagnose(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  using luchs_sdk::Protocol;
  const auto s = chassis_->state();
  const auto st = chassis_->status();
  const Protocol p = opt_.protocol_type;
  stat.add("chassis", std::string("Luchs ") + luchs_sdk::build_name(opt_.geometry.build));
  stat.add("protocol", luchs_sdk::protocol_name(p));
  stat.add("interface", opt_.interface);
  stat.add("dry run", dry_run_);
  stat.add("feedback age [s]", st.feedback_age_s);
  stat.add("command age [s]", st.command_age_s);
  stat.add("command limited", st.limited);
  stat.add("motion model (commanded)", static_cast<int>(st.model));
  stat.add("gear (commanded)", static_cast<int>(st.gear));
  stat.add("control source", s.motion.source == luchs_sdk::ControlSource::kHost ? "host" :
    s.motion.source == luchs_sdk::ControlSource::kRemoteControl ? "remote" : "unknown");
  stat.add("battery [%]", s.battery.soc_percent);
  stat.add("battery [V]", s.battery.voltage_v);
  stat.add("CAN tx cycles", st.tx_cycles);
  stat.add("CAN tx errors", st.tx_errors);
  stat.add("frames received", s.frames_received);
  if (p == Protocol::kLegacyA) {
    stat.add("fault", luchs_sdk::fault_code_text(s.safety.fault_code));
  } else if (p == Protocol::kLci) {
    static const char * kMode[] = {"maintenance", "remote control", "automatic"};
    stat.add("operating mode", s.status.operating_mode < 3 ? kMode[s.status.operating_mode] : "?");
    stat.add("fault level", static_cast<int>(s.status.fault_level));
    stat.add("fault code", luchs_sdk::fault_code_text(s.safety.fault_code));
    stat.add("e-stop source", luchs_sdk::estop_source_text(s.status.estop_source_bits));
    stat.add("limits", luchs_sdk::limit_reason_text(s.limits.reason_bits));
    stat.add("max speed fwd / rev [m/s]", std::to_string(s.limits.max_vx_fwd) + " / " +
      std::to_string(s.limits.max_vx_rev));
    stat.add("park brake", s.status.park_brake);
    stat.add("EPB / EPS front / EPS rear / brake", std::to_string(s.brake.epb_state) + " / " +
      std::to_string(s.brake.eps_front) + " / " + std::to_string(s.brake.eps_rear) + " / " +
      std::to_string(s.brake.brake_state));
    stat.add("heartbeat counter", static_cast<int>(st.heartbeat_counter));
    stat.add("chassis checksum errors", s.status.checksum_errors);
    stat.add("chassis counter errors", s.status.counter_errors);
    if (s.version_stamp) {
      stat.add("chassis model", luchs_sdk::model_id_text(s.version.model_id));
      stat.add("LCI version", std::to_string(s.version.lci_major) + "." +
        std::to_string(s.version.lci_minor));
    }
  } else {
    stat.add("fault", luchs_sdk::legacy_b::fault_text(s.status.fault_level));
    stat.add("throttle / brake [%]", std::to_string(st.pedals.throttle) + " / " +
      std::to_string(st.pedals.brake));
  }

  const bool lci = p == Protocol::kLci;
  if (!dry_run_ && !st.feedback_ok) {
    stat.summary(DiagnosticStatus::ERROR, "no chassis feedback: check power, CAN wiring and bitrate");
  } else if (lci && st.model_mismatch) {
    stat.summary(DiagnosticStatus::ERROR, "chassis identifies as " +
      luchs_sdk::model_id_text(s.version.model_id) + ": check the build parameter");
  } else if (s.safety.estop_raw != 0 || (p != Protocol::kLegacyB && s.safety.fault_code == 0x02)) {
    stat.summary(DiagnosticStatus::ERROR, "emergency stop pressed");
  } else if (lci && s.status.estop_source_bits != 0) {
    stat.summary(DiagnosticStatus::ERROR, "emergency stop: " +
      luchs_sdk::estop_source_text(s.status.estop_source_bits));
  } else if (lci && s.status.fault_level >= 3) {
    stat.summary(DiagnosticStatus::ERROR, "chassis fault: stop");
  } else if (p == Protocol::kLegacyA && s.safety.fault_code >= 0x05) {
    stat.summary(DiagnosticStatus::ERROR, luchs_sdk::fault_code_text(s.safety.fault_code));
  } else if (p == Protocol::kLegacyB && s.status.fault_level != 0) {
    stat.summary(DiagnosticStatus::ERROR, luchs_sdk::legacy_b::fault_text(s.status.fault_level));
  } else if (lci && (s.brake.eps_front >= 2 || s.brake.eps_rear >= 2 || s.brake.brake_state >= 2 ||
    s.brake.epb_state == 4))
  {
    stat.summary(DiagnosticStatus::ERROR, "brake / parking brake / steering actuator degraded");
  } else if (st.tx_errors > 0 && st.tx_cycles > 0 && st.tx_errors * 10 > st.tx_cycles) {
    stat.summary(DiagnosticStatus::WARN, "CAN transmit errors");
  } else if (lci && s.status.checksum_errors > 0 && s.status.checksum_errors * 100 > s.frames_received) {
    stat.summary(DiagnosticStatus::WARN, "chassis-state checksum errors");
  } else if (s.battery_stamp && s.battery.soc_percent < battery_low_percent_) {
    stat.summary(DiagnosticStatus::WARN, "battery low");
  } else if (lci && s.status.fault_level >= 1) {
    stat.summary(DiagnosticStatus::WARN, s.status.fault_level == 2 ? "chassis derated" :
      "chassis warning");
  } else if (p == Protocol::kLegacyA && s.safety.fault_code != 0) {
    stat.summary(DiagnosticStatus::WARN, luchs_sdk::fault_code_text(s.safety.fault_code));
  } else if (s.motion.source == luchs_sdk::ControlSource::kRemoteControl) {
    stat.summary(DiagnosticStatus::OK, "remote control has priority (mode selector)");
  } else if (lci && st.live_limits_applied) {
    stat.summary(DiagnosticStatus::OK, "ok, chassis limits active: " +
      luchs_sdk::limit_reason_text(s.limits.reason_bits));
  } else {
    stat.summary(DiagnosticStatus::OK, "ok");
  }
}

}  // namespace luchs_driver

RCLCPP_COMPONENTS_REGISTER_NODE(luchs_driver::LuchsDriverNode)
