// Copyright 2026 TerraAgent Robotics. Chassis: RJ Tech Limited.
// SPDX-License-Identifier: Apache-2.0
//
// ROS 1 driver for Luchs A and Luchs B. Same luchs_sdk, topics, frames and services as the ROS 2
// driver (luchs_ros2); parameter names use '/' where ROS 2 uses '.' (geometry/wheelbase).

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <luchs_msgs/ChassisState.h>
#include <luchs_msgs/DriveCommand.h>
#include <luchs_msgs/LampCommand.h>
#include <luchs_msgs/SafetyState.h>
#include <luchs_msgs/SetMotionModel.h>
#include <luchs_msgs/SetPayloadPower.h>
#include <luchs_msgs/SetSpeedLimit.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "luchs_sdk/chassis.hpp"
#include "luchs_sdk/sim_bus.hpp"

namespace
{
using luchs_sdk::Build;
using luchs_sdk::MotionModel;
using luchs_sdk::Protocol;
namespace la = luchs_sdk::legacy_a;

const char * kWheelNames[4] = {"front_left", "front_right", "rear_left", "rear_right"};

template<typename T>
T param(ros::NodeHandle & n, const std::string & name, const T & def)
{
  T v;
  n.param<T>(name, v, def);
  return v;
}

double age(const std::optional<luchs_sdk::Clock::time_point> & t)
{
  return t ? std::chrono::duration<double>(luchs_sdk::Clock::now() - *t).count() : 1e9;
}

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

Protocol parse_protocol(const std::string & s, Build b)
{
  if (s.empty()) {return b == Build::kLuchsB ? Protocol::kLci : Protocol::kLegacyA;}
  if (s == "a") {return Protocol::kLegacyA;}
  if (s == "lci") {return Protocol::kLci;}
  if (s == "legacy_b") {return Protocol::kLegacyB;}
  throw std::invalid_argument("protocol/type must be a, lci or legacy_b (got '" + s + "')");
}

MotionModel parse_model(const std::string & s)
{
  if (s == "diff") {return MotionModel::kDifferential;}
  if (s == "ackermann") {return MotionModel::kFrontAckermann;}
  if (s == "dual_ackermann") {return MotionModel::kDualAckermann;}
  if (s == "4ws") {return MotionModel::kFourWheelSteer;}
  throw std::invalid_argument("motion_model must be one of diff, ackermann, dual_ackermann, 4ws");
}

la::WheelOrder parse_order(const std::string & s)
{
  if (s == "lf_rf_lr_rr") {return la::WheelOrder::kLfRfLrRr;}
  if (s == "lf_lr_rf_rr") {return la::WheelOrder::kLfLrRfRr;}
  throw std::invalid_argument("wheel order must be lf_rf_lr_rr or lf_lr_rf_rr");
}

bool steers(Build b, std::size_t wheel)
{
  switch (b) {
    case Build::kSkidSteer: return false;
    case Build::kFrontAckermann: return wheel < 2;
    default: return true;
  }
}

luchs_sdk::ChassisOptions read_options(ros::NodeHandle & n)
{
  const Build b = parse_build(param<std::string>(n, "build", "4ws"));
  luchs_sdk::ChassisOptions o =
    luchs_sdk::ChassisOptions::for_build(b, parse_protocol(param<std::string>(n, "protocol/type", ""), b));
  o.interface = param<std::string>(n, "can_interface", "can0");
  const std::string model = param<std::string>(n, "motion_model", "");
  if (!model.empty()) {o.model = parse_model(model);}

  auto & g = o.geometry;
  g.wheelbase = param(n, "geometry/wheelbase", g.wheelbase);
  g.track = param(n, "geometry/track", g.track);
  g.wheel_radius = param(n, "geometry/wheel_radius", g.wheel_radius);
  g.max_steer_rad = param(n, "geometry/max_steer_angle", g.max_steer_rad);
  g.max_vx = param(n, "limits/max_vx", g.max_vx);
  g.max_vy = param(n, "limits/max_vy", g.max_vy);
  g.max_wz = param(n, "limits/max_wz", g.max_wz);
  g.allow_pivot_turn = param(n, "limits/allow_pivot_turn", g.allow_pivot_turn);
  o.max_accel_x = param(n, "limits/max_accel_x", o.max_accel_x);
  o.max_accel_y = param(n, "limits/max_accel_y", o.max_accel_y);
  o.max_accel_wz = param(n, "limits/max_accel_wz", o.max_accel_wz);

  o.tx_rate_hz = param(n, "control/rate", o.tx_rate_hz);
  o.cmd_timeout_s = param(n, "control/cmd_timeout", o.cmd_timeout_s);
  o.feedback_timeout_s = param(n, "control/feedback_timeout", o.feedback_timeout_s);
  o.park_when_idle = param(n, "control/park_when_idle", o.park_when_idle);
  o.send_host_time = param(n, "control/send_host_time", o.send_host_time);

  auto & p = o.protocol;
  p.wheel_steer_cmd_id = static_cast<uint32_t>(param<int>(n, "protocol/wheel_steer_cmd_id",
      static_cast<int>(p.wheel_steer_cmd_id)));
  if (p.wheel_steer_cmd_id != la::kId5A2 && p.wheel_steer_cmd_id != la::kId5A3) {
    throw std::invalid_argument("protocol/wheel_steer_cmd_id must be 1442 (0x5A2) or 1443 (0x5A3)");
  }
  p.wheel_steer_cmd_order = parse_order(param<std::string>(n, "protocol/wheel_steer_cmd_order",
      "lf_rf_lr_rr"));
  p.wheel_steer_fb_order = parse_order(param<std::string>(n, "protocol/wheel_steer_fb_order",
      "lf_lr_rf_rr"));
  p.wheel_speed_fb_order = parse_order(param<std::string>(n, "protocol/wheel_speed_fb_order",
      "lf_lr_rf_rr"));
  p.wheel_speed_scale = param(n, "protocol/wheel_speed_scale", p.wheel_speed_scale);
  p.ultrasonic_scale_m = param(n, "protocol/ultrasonic_scale", p.ultrasonic_scale_m);
  p.battery_soc_scale = param(n, "protocol/battery_soc_scale", p.battery_soc_scale);
  p.battery_temp_scale = param(n, "protocol/battery_temperature_scale", p.battery_temp_scale);
  p.battery_voltage_scale = param(n, "protocol/battery_voltage_scale", p.battery_voltage_scale);
  p.charging_scale = param(n, "protocol/charging_scale", p.charging_scale);

  o.follow_live_limits = param(n, "lci/follow_live_limits", o.follow_live_limits);
  o.lci.accel_limit = param(n, "lci/accel_limit", o.lci.accel_limit);
  o.lci.check_checksum = param(n, "lci/check_checksum", o.lci.check_checksum);

  auto & lb = o.legacy_b;
  lb.cmd_id = static_cast<uint32_t>(param<int>(n, "legacy_b/cmd_id", static_cast<int>(lb.cmd_id)));
  lb.fb_id = static_cast<uint32_t>(param<int>(n, "legacy_b/fb_id", static_cast<int>(lb.fb_id)));
  lb.steer_sign = param<int>(n, "legacy_b/steer_sign", lb.steer_sign);
  if (lb.steer_sign != 1 && lb.steer_sign != -1) {
    throw std::invalid_argument("legacy_b/steer_sign must be 1 or -1");
  }
  lb.max_wheel_angle_deg = param(n, "legacy_b/max_wheel_angle_deg", lb.max_wheel_angle_deg);
  lb.steer_rate_dps = param(n, "legacy_b/steer_rate", lb.steer_rate_dps);
  lb.speed_cap_kmh = param(n, "legacy_b/speed_cap_kmh", lb.speed_cap_kmh);
  lb.throttle_ff = param(n, "legacy_b/throttle_ff", lb.throttle_ff);
  lb.throttle_kp = param(n, "legacy_b/throttle_kp", lb.throttle_kp);
  lb.throttle_ki = param(n, "legacy_b/throttle_ki", lb.throttle_ki);
  lb.max_throttle = param(n, "legacy_b/max_throttle", lb.max_throttle);
  lb.brake_kp = param(n, "legacy_b/brake_kp", lb.brake_kp);
  lb.hold_brake = param(n, "legacy_b/hold_brake", lb.hold_brake);
  lb.max_brake = param(n, "legacy_b/max_brake", lb.max_brake);
  return o;
}

class LuchsDriver
{
public:
  LuchsDriver(ros::NodeHandle & nh, ros::NodeHandle & pnh)
  : nh_(nh), pnh_(pnh)
  {
    opt_ = read_options(pnh_);
    dry_run_ = param(pnh_, "dry_run", false);
    odom_frame_ = param<std::string>(pnh_, "odom/frame_id", "odom");
    base_frame_ = param<std::string>(pnh_, "odom/child_frame_id", "base_footprint");
    publish_tf_ = param(pnh_, "odom/publish_tf", true);
    joint_prefix_ = param<std::string>(pnh_, "joint_prefix", "");
    imu_frame_ = param<std::string>(pnh_, "imu/frame_id", "base_link");
    battery_low_percent_ = param(pnh_, "battery_low_percent", 15.0);
    battery_capacity_ah_ = param(pnh_, "battery/design_capacity",
        opt_.geometry.build == Build::kLuchsB ? 60.0 : 15.0);
    const double rate = param(pnh_, "publish_rate", 50.0);
    const bool stamped = param(pnh_, "cmd_vel_stamped", false);
    const double speed_limit = param(pnh_, "lci/speed_limit", 0.0);

    chassis_ = dry_run_ ?
      std::make_unique<luchs_sdk::Chassis>(opt_, luchs_sdk::make_simulator(opt_)) :
      std::make_unique<luchs_sdk::Chassis>(opt_);
    chassis_->set_speed_limit(speed_limit);
    chassis_->on_feedback([this](const luchs_sdk::ChassisState & s, uint32_t id) {
        on_feedback(s, id);
      });

    pub_odom_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
    pub_state_ = pnh_.advertise<luchs_msgs::ChassisState>("state", 10);
    pub_safety_ = pnh_.advertise<luchs_msgs::SafetyState>("safety", 10);
    pub_battery_ = nh_.advertise<sensor_msgs::BatteryState>("battery_state", 10);
    pub_joints_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);
    if (opt_.protocol_type == Protocol::kLci) {
      pub_imu_ = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 50);
    }
    if (publish_tf_) {tf_ = std::make_unique<tf2_ros::TransformBroadcaster>();}

    if (stamped) {
      sub_cmd_ = nh_.subscribe<geometry_msgs::TwistStamped>("cmd_vel", 10,
          [this](const geometry_msgs::TwistStamped::ConstPtr & m) {on_cmd_vel(m->twist);});
    } else {
      sub_cmd_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10,
          [this](const geometry_msgs::Twist::ConstPtr & m) {on_cmd_vel(*m);});
    }
    sub_drive_ = pnh_.subscribe<luchs_msgs::DriveCommand>("drive_command", 10,
        [this](const luchs_msgs::DriveCommand::ConstPtr & m) {on_drive_command(*m);});
    sub_lamps_ = pnh_.subscribe<luchs_msgs::LampCommand>("lamps", 10,
        [this](const luchs_msgs::LampCommand::ConstPtr & m) {on_lamps(*m);});

    srv_reset_ = pnh_.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
      "reset_odometry", [this](std_srvs::Trigger::Request &, std_srvs::Trigger::Response & res) {
        std::lock_guard<std::mutex> lk(odom_mtx_);
        odom_.reset();
        res.success = true;
        res.message = "odometry reset to zero";
        return true;
      });
    srv_model_ = pnh_.advertiseService<luchs_msgs::SetMotionModel::Request,
        luchs_msgs::SetMotionModel::Response>("set_motion_model",
        [this](luchs_msgs::SetMotionModel::Request & req, luchs_msgs::SetMotionModel::Response & res) {
          const auto m = static_cast<MotionModel>(req.motion_model);
          if (req.motion_model < 1 || req.motion_model > 4 ||
          !luchs_sdk::model_supported(opt_.geometry.build, m))
          {
            res.success = false;
            res.message = "motion model not supported by this build";
            return true;
          }
          chassis_->set_model(m);
          res.success = true;
          res.message = "motion model set";
          return true;
        });
    const bool heartbeat = opt_.protocol_type != Protocol::kLegacyA;
    const bool lci = opt_.protocol_type == Protocol::kLci;
    srv_estop_ = pnh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
      "estop", [this, heartbeat](std_srvs::SetBool::Request & req, std_srvs::SetBool::Response & res) {
        res.success = heartbeat;
        if (!heartbeat) {
          res.message = "not available on the Luchs A protocol: use the emergency stop button";
          return true;
        }
        chassis_->request_estop(req.data);
        res.message = req.data ?
        "emergency stop requested (latched by the chassis until a maintenance reset)" :
        "emergency stop request cleared";
        return true;
      });
    srv_park_ = pnh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
      "park", [this, heartbeat](std_srvs::SetBool::Request & req, std_srvs::SetBool::Response & res) {
        res.success = heartbeat;
        if (!heartbeat) {
          res.message = "not available on the Luchs A protocol (gear P is engaged when idle)";
          return true;
        }
        chassis_->request_park(req.data);
        res.message = req.data ? "park requested" : "park request cleared";
        return true;
      });
    srv_payload_ = pnh_.advertiseService<luchs_msgs::SetPayloadPower::Request,
        luchs_msgs::SetPayloadPower::Response>("set_payload_power",
        [this, lci](luchs_msgs::SetPayloadPower::Request & req, luchs_msgs::SetPayloadPower::Response & res) {
          res.success = lci && (req.output == 1 || req.output == 2);
          if (!res.success) {
            res.message = lci ? "output must be 1 or 2" : "LCI only";
            return true;
          }
          chassis_->set_payload_power(req.output, req.on);
          res.message = "payload power request sent";
          return true;
        });
    srv_speed_ = pnh_.advertiseService<luchs_msgs::SetSpeedLimit::Request,
        luchs_msgs::SetSpeedLimit::Response>("set_speed_limit",
        [this, lci](luchs_msgs::SetSpeedLimit::Request & req, luchs_msgs::SetSpeedLimit::Response & res) {
          res.success = lci && req.speed_limit >= 0.0;
          if (!res.success) {
            res.message = lci ? "speed_limit must be >= 0" : "LCI only (use limits/max_vx)";
            return true;
          }
          chassis_->set_speed_limit(req.speed_limit);
          res.message = "speed limit sent in the heartbeat";
          return true;
        });

    diag_.setHardwareID("luchs:" + opt_.interface);
    diag_.add("chassis", this, &LuchsDriver::diagnose);

    if (!chassis_->start()) {
      ROS_FATAL("cannot open CAN interface '%s': %s", opt_.interface.c_str(),
        chassis_->last_error().c_str());
      throw std::runtime_error(chassis_->last_error());
    }
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::clamp(rate, 1.0, 200.0)),
        [this](const ros::TimerEvent &) {publish(); diag_.update();});
    static const char * kModel[] = {"?", "differential", "front Ackermann", "four-wheel steer",
      "dual Ackermann"};
    ROS_INFO("Luchs driver %s (ROS 1) on %s: Luchs %s, %s, %s, %.0f Hz, cmd timeout %.2f s%s",
      luchs_sdk::kSdkVersion, opt_.interface.c_str(), luchs_sdk::build_name(opt_.geometry.build),
      luchs_sdk::protocol_name(opt_.protocol_type),
      kModel[static_cast<int>(chassis_->status().model)], opt_.tx_rate_hz, opt_.cmd_timeout_s,
      dry_run_ ? " (DRY RUN: built-in simulator, no CAN)" : "");
    if (opt_.protocol_type == Protocol::kLegacyB) {
      ROS_WARN("Luchs B VCU upper interface: pedal-level control with 1 km/h speed feedback and no "
        "steering feedback. For bring-up and teleoperation; use LCI for autonomy.");
    }
  }

  ~LuchsDriver()
  {
    if (chassis_) {chassis_->stop();}
  }

private:
  void on_cmd_vel(const geometry_msgs::Twist & m)
  {
    chassis_->set_twist({m.linear.x, m.linear.y, m.angular.z});
  }

  void on_drive_command(const luchs_msgs::DriveCommand & m)
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

  void on_lamps(const luchs_msgs::LampCommand & m)
  {
    luchs_sdk::LampCommand l;
    const uint8_t max_turn = opt_.protocol_type == Protocol::kLci ? 3 : 2;
    l.turn = static_cast<luchs_sdk::TurnSignal>(std::min<uint8_t>(m.turn_signal, max_turn));
    l.reverse = m.reverse_lamp;
    l.fault = m.fault_lamp;
    l.headlight = std::min<uint8_t>(m.headlight, 2);
    l.horn = m.horn;
    l.beacon = m.beacon;
    chassis_->set_lamps(l);
  }

  void on_feedback(const luchs_sdk::ChassisState & s, uint32_t id)
  {
    if (id != luchs_sdk::motion_feedback_id(opt_) || !s.motion_stamp) {return;}
    std::lock_guard<std::mutex> lk(odom_mtx_);
    if (last_motion_) {
      odom_.update(s.motion.twist, std::chrono::duration<double>(*s.motion_stamp - *last_motion_).count());
    }
    last_motion_ = s.motion_stamp;
  }

  void publish()
  {
    const auto s = chassis_->state();
    const auto st = chassis_->status();
    const ros::Time now = ros::Time::now();
    const double dt = last_publish_.isZero() ? 0.0 : (now - last_publish_).toSec();
    last_publish_ = now;

    double x, y, yaw;
    {
      std::lock_guard<std::mutex> lk(odom_mtx_);
      x = odom_.x();
      y = odom_.y();
      yaw = odom_.yaw();
    }
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    nav_msgs::Odometry od;
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
    const bool still = std::abs(s.motion.twist.vx) < 1e-3 && std::abs(s.motion.twist.vy) < 1e-3 &&
      std::abs(s.motion.twist.wz) < 1e-3;
    const double pv = still ? 1e-6 : 1e-2, yv = still ? 1e-6 : 3e-2, big = 1e6;
    od.pose.covariance = {pv, 0, 0, 0, 0, 0, 0, pv, 0, 0, 0, 0, 0, 0, big, 0, 0, 0,
      0, 0, 0, big, 0, 0, 0, 0, 0, 0, big, 0, 0, 0, 0, 0, 0, yv};
    od.twist.covariance = {1e-3, 0, 0, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 0, 0, big, 0, 0, 0,
      0, 0, 0, big, 0, 0, 0, 0, 0, 0, big, 0, 0, 0, 0, 0, 0, 2e-3};
    pub_odom_.publish(od);
    if (tf_) {
      geometry_msgs::TransformStamped t;
      t.header = od.header;
      t.child_frame_id = base_frame_;
      t.transform.translation.x = x;
      t.transform.translation.y = y;
      t.transform.rotation = od.pose.pose.orientation;
      tf_->sendTransform(t);
    }

    luchs_msgs::ChassisState cs;
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
      cs.motor_temperature[i] = s.temperature.motor[i];
      cs.controller_temperature[i] = s.temperature.controller[i];
    }
    cs.fault_code = s.safety.fault_code;
    cs.fault_text = opt_.protocol_type == Protocol::kLegacyB ?
      luchs_sdk::legacy_b::fault_text(s.status.fault_level) :
      luchs_sdk::fault_code_text(s.safety.fault_code);
    cs.motor_fault_high = s.motor_fault_high;
    cs.motor_fault_low = s.motor_fault_low;
    cs.turn_signal = s.lamps.turn_raw;
    cs.reverse_lamp = s.lamps.reverse_raw != 0;
    cs.fault_lamp = s.lamps.fault_raw != 0;
    cs.light_rain_raw = s.lamps.light_rain_raw;
    cs.lci_ok = opt_.protocol_type == Protocol::kLci && age(s.status_stamp) < 0.5;
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
    if (s.version_stamp) {
      cs.model_id = s.version.model_id;
      cs.model_text = luchs_sdk::model_id_text(s.version.model_id);
      cs.firmware_version = std::to_string(s.version.fw_major) + "." +
        std::to_string(s.version.fw_minor) + "." + std::to_string(s.version.fw_patch);
      cs.lci_version = std::to_string(s.version.lci_major) + "." + std::to_string(s.version.lci_minor);
    }
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
    pub_state_.publish(cs);

    luchs_msgs::SafetyState sf;
    sf.header = cs.header;
    sf.estop_left = s.safety.estop_raw == 1;
    sf.estop_right = s.safety.estop_raw == 2;
    sf.bumper_front = s.safety.bumper_raw == 1;
    sf.bumper_rear = s.safety.bumper_raw == 2;
    sf.ultrasonic_trigger = s.safety.ultrasonic_trigger_raw;
    for (std::size_t i = 0; i < 4; ++i) {sf.ultrasonic_range[i] = static_cast<float>(s.safety.ultrasonic_m[i]);}
    sf.fault_code = cs.fault_code;
    sf.fault_text = cs.fault_text;
    pub_safety_.publish(sf);

    if (pub_imu_ && age(s.imu_rate_stamp) < 0.1) {
      sensor_msgs::Imu imu;
      imu.header.stamp = now;
      imu.header.frame_id = joint_prefix_ + imu_frame_;
      imu.orientation_covariance[0] = -1.0;
      imu.angular_velocity.x = s.imu.rate[0];
      imu.angular_velocity.y = s.imu.rate[1];
      imu.angular_velocity.z = s.imu.rate[2];
      imu.linear_acceleration.x = s.imu.accel[0];
      imu.linear_acceleration.y = s.imu.accel[1];
      imu.linear_acceleration.z = s.imu.accel[2];
      imu.angular_velocity_covariance = {1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4};
      imu.linear_acceleration_covariance = {1e-2, 0, 0, 0, 1e-2, 0, 0, 0, 1e-2};
      pub_imu_.publish(imu);
    }

    if (s.battery_stamp) {
      sensor_msgs::BatteryState b;
      b.header = cs.header;
      b.voltage = static_cast<float>(s.battery.voltage_v);
      b.temperature = static_cast<float>(s.battery.temperature_c);
      b.current = std::numeric_limits<float>::quiet_NaN();
      b.charge = std::numeric_limits<float>::quiet_NaN();
      b.capacity = std::numeric_limits<float>::quiet_NaN();
      b.design_capacity = static_cast<float>(battery_capacity_ah_);
      b.percentage = static_cast<float>(std::clamp(s.battery.soc_percent / 100.0, 0.0, 1.0));
      const bool charging = opt_.protocol_type == Protocol::kLegacyA ? s.safety.fault_code == 0x04 :
        s.battery.charging_percent > 0.0;
      b.power_supply_status = charging ? sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING :
        sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
      b.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE;
      b.present = true;
      pub_battery_.publish(b);
    }

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
    if (!s.wheel_speed_stamp) {
      const auto sol = luchs_sdk::solve(s.motion.twist, st.model, opt_.geometry);
      for (std::size_t i = 0; i < 4; ++i) {spin[i] = sol.wheel_speed[i] / opt_.geometry.wheel_radius;}
    }
    sensor_msgs::JointState js;
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
    pub_joints_.publish(js);
  }

  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    using diagnostic_msgs::DiagnosticStatus;
    const auto s = chassis_->state();
    const auto st = chassis_->status();
    const Protocol p = opt_.protocol_type;
    const bool lci = p == Protocol::kLci;
    stat.add("chassis", std::string("Luchs ") + luchs_sdk::build_name(opt_.geometry.build));
    stat.add("protocol", luchs_sdk::protocol_name(p));
    stat.add("interface", opt_.interface);
    stat.add("dry run", dry_run_);
    stat.add("feedback age [s]", st.feedback_age_s);
    stat.add("command age [s]", st.command_age_s);
    stat.add("command limited", st.limited);
    stat.add("battery [%]", s.battery.soc_percent);
    stat.add("CAN tx cycles", st.tx_cycles);
    stat.add("CAN tx errors", st.tx_errors);
    if (lci) {
      stat.add("e-stop source", luchs_sdk::estop_source_text(s.status.estop_source_bits));
      stat.add("limits", luchs_sdk::limit_reason_text(s.limits.reason_bits));
      stat.add("heartbeat counter", static_cast<int>(st.heartbeat_counter));
      stat.add("chassis checksum errors", s.status.checksum_errors);
      if (s.version_stamp) {stat.add("chassis model", luchs_sdk::model_id_text(s.version.model_id));}
    }
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
    } else if (st.tx_errors > 0 && st.tx_cycles > 0 && st.tx_errors * 10 > st.tx_cycles) {
      stat.summary(DiagnosticStatus::WARN, "CAN transmit errors");
    } else if (s.battery_stamp && s.battery.soc_percent < battery_low_percent_) {
      stat.summary(DiagnosticStatus::WARN, "battery low");
    } else if (lci && s.status.fault_level >= 1) {
      stat.summary(DiagnosticStatus::WARN, s.status.fault_level == 2 ? "chassis derated" : "chassis warning");
    } else if (s.motion.source == luchs_sdk::ControlSource::kRemoteControl) {
      stat.summary(DiagnosticStatus::OK, "remote control has priority (mode selector)");
    } else {
      stat.summary(DiagnosticStatus::OK, "ok");
    }
  }

  ros::NodeHandle nh_, pnh_;
  luchs_sdk::ChassisOptions opt_;
  std::unique_ptr<luchs_sdk::Chassis> chassis_;
  bool dry_run_{false};
  std::mutex odom_mtx_;
  luchs_sdk::Odometry odom_;
  std::optional<luchs_sdk::Clock::time_point> last_motion_;
  luchs_sdk::PerWheel wheel_pos_{};
  ros::Time last_publish_;
  std::string odom_frame_, base_frame_, joint_prefix_, imu_frame_;
  bool publish_tf_{true};
  double battery_low_percent_{15.0}, battery_capacity_ah_{15.0};
  ros::Publisher pub_odom_, pub_state_, pub_safety_, pub_battery_, pub_joints_, pub_imu_;
  ros::Subscriber sub_cmd_, sub_drive_, sub_lamps_;
  ros::ServiceServer srv_reset_, srv_model_, srv_estop_, srv_park_, srv_payload_, srv_speed_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  diagnostic_updater::Updater diag_;
  ros::Timer timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "luchs_driver");
  ros::NodeHandle nh, pnh("~");
  try {
    LuchsDriver driver(nh, pnh);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception & e) {
    ROS_FATAL("%s", e.what());
    return 1;
  }
  return 0;
}
