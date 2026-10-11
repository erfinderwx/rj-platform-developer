// Copyright 2026 TerraAgent Robotics. Chassis: RJ Tech Limited.
// SPDX-License-Identifier: Apache-2.0

#ifndef LUCHS_DRIVER__LUCHS_DRIVER_NODE_HPP_
#define LUCHS_DRIVER__LUCHS_DRIVER_NODE_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "luchs_msgs/msg/chassis_state.hpp"
#include "luchs_msgs/msg/drive_command.hpp"
#include "luchs_msgs/msg/lamp_command.hpp"
#include "luchs_msgs/msg/safety_state.hpp"
#include "luchs_msgs/srv/set_motion_model.hpp"
#include "luchs_msgs/srv/set_payload_power.hpp"
#include "luchs_msgs/srv/set_speed_limit.hpp"
#include "luchs_sdk/chassis.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace luchs_driver
{

/// Parses a build name ("df", "fa", "da", "4ws" for Luchs A, "b" for Luchs B).
/// Throws std::invalid_argument.
luchs_sdk::Build parse_build(const std::string & s);
/// Parses a protocol name ("a", "lci", "legacy_b"); empty = default for the build.
luchs_sdk::Protocol parse_protocol(const std::string & s, luchs_sdk::Build b);
/// Parses a motion model name ("diff", "ackermann", "dual_ackermann", "4ws").
luchs_sdk::MotionModel parse_model(const std::string & s);
/// Parses a wheel order name ("lf_rf_lr_rr", "lf_lr_rf_rr").
luchs_sdk::legacy_a::WheelOrder parse_order(const std::string & s);

/// Builds SDK options from the node's parameters (declares them with defaults).
luchs_sdk::ChassisOptions declare_options(rclcpp::Node & node);

class LuchsDriverNode : public rclcpp::Node
{
public:
  explicit LuchsDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LuchsDriverNode() override;

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist & msg);
  void on_drive_command(const luchs_msgs::msg::DriveCommand & msg);
  void on_lamps(const luchs_msgs::msg::LampCommand & msg);
  void on_feedback(const luchs_sdk::ChassisState & s, uint32_t id);
  void publish();
  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & stat);

  luchs_sdk::ChassisOptions opt_;
  std::unique_ptr<luchs_sdk::Chassis> chassis_;
  bool dry_run_{false};

  // odometry (integrated from chassis-measured twist, in the receive thread)
  std::mutex odom_mtx_;
  luchs_sdk::Odometry odom_;
  std::optional<luchs_sdk::Clock::time_point> last_motion_;
  luchs_sdk::PerWheel wheel_pos_{};
  std::optional<rclcpp::Time> last_publish_;

  std::string odom_frame_, base_frame_, joint_prefix_, imu_frame_;
  bool publish_tf_{true};
  double battery_low_percent_{15.0};
  double battery_capacity_ah_{15.0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_vel_stamped_;
  rclcpp::Subscription<luchs_msgs::msg::DriveCommand>::SharedPtr sub_drive_;
  rclcpp::Subscription<luchs_msgs::msg::LampCommand>::SharedPtr sub_lamps_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<luchs_msgs::msg::ChassisState>::SharedPtr pub_state_;
  rclcpp::Publisher<luchs_msgs::msg::SafetyState>::SharedPtr pub_safety_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joints_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_estop_, srv_park_;
  rclcpp::Service<luchs_msgs::srv::SetPayloadPower>::SharedPtr srv_payload_;
  rclcpp::Service<luchs_msgs::srv::SetSpeedLimit>::SharedPtr srv_speed_limit_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_odom_;
  rclcpp::Service<luchs_msgs::srv::SetMotionModel>::SharedPtr srv_model_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::unique_ptr<diagnostic_updater::Updater> diag_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace luchs_driver

#endif  // LUCHS_DRIVER__LUCHS_DRIVER_NODE_HPP_
