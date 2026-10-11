// Copyright 2026 TerraAgent Robotics. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include "luchs_driver/luchs_driver_node.hpp"

class DriverTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(DriverTest, DryRunStartsWithDefaults)
{
  rclcpp::NodeOptions o;
  o.append_parameter_override("dry_run", true);
  auto n = std::make_shared<luchs_driver::LuchsDriverNode>(o);
  EXPECT_EQ(n->get_parameter("build").as_string(), "4ws");
  EXPECT_DOUBLE_EQ(n->get_parameter("geometry.track").as_double(), 0.5);
  EXPECT_EQ(n->get_parameter("protocol.wheel_steer_cmd_id").as_int(), 0x5A3);
  EXPECT_EQ(n->count_subscribers("/odom"), 0u);
  EXPECT_EQ(n->count_publishers("/odom"), 1u);
  EXPECT_EQ(n->count_publishers("/luchs_driver/state"), 1u);
}

TEST_F(DriverTest, BuildSetsGeometry)
{
  rclcpp::NodeOptions o;
  o.append_parameter_override("dry_run", true);
  o.append_parameter_override("build", "fa");
  auto n = std::make_shared<luchs_driver::LuchsDriverNode>(o);
  EXPECT_DOUBLE_EQ(n->get_parameter("geometry.track").as_double(), 0.596);
  EXPECT_NEAR(n->get_parameter("geometry.max_steer_angle").as_double(), 0.6, 1e-9);
}

TEST_F(DriverTest, RejectsBadValues)
{
  rclcpp::NodeOptions o;
  o.append_parameter_override("dry_run", true);
  o.append_parameter_override("build", "x9");
  EXPECT_THROW(std::make_shared<luchs_driver::LuchsDriverNode>(o), std::invalid_argument);
  rclcpp::NodeOptions p;
  p.append_parameter_override("dry_run", true);
  p.append_parameter_override("protocol.wheel_steer_cmd_id", 0x123);
  EXPECT_THROW(std::make_shared<luchs_driver::LuchsDriverNode>(p), std::invalid_argument);
}

TEST(Parse, Names)
{
  EXPECT_EQ(luchs_driver::parse_model("dual_ackermann"), luchs_sdk::MotionModel::kDualAckermann);
  EXPECT_THROW(luchs_driver::parse_model("mecanum"), std::invalid_argument);
  EXPECT_EQ(luchs_driver::parse_order("lf_rf_lr_rr"), luchs_sdk::legacy_a::WheelOrder::kLfRfLrRr);
}

TEST_F(DriverTest, LuchsBDefaultsToLci)
{
  rclcpp::NodeOptions o;
  o.append_parameter_override("dry_run", true);
  o.append_parameter_override("build", "b");
  auto n = std::make_shared<luchs_driver::LuchsDriverNode>(o);
  EXPECT_DOUBLE_EQ(n->get_parameter("geometry.wheelbase").as_double(), 1.43);
  EXPECT_DOUBLE_EQ(n->get_parameter("geometry.track").as_double(), 0.97);
  EXPECT_DOUBLE_EQ(n->get_parameter("control.rate").as_double(), 50.0);
  EXPECT_EQ(n->count_publishers("/imu/data_raw"), 1u);
  EXPECT_EQ(n->count_publishers("/luchs_driver/state"), 1u);
}

TEST_F(DriverTest, LuchsBVcuInterface)
{
  rclcpp::NodeOptions o;
  o.append_parameter_override("dry_run", true);
  o.append_parameter_override("build", "b");
  o.append_parameter_override("protocol.type", "legacy_b");
  auto n = std::make_shared<luchs_driver::LuchsDriverNode>(o);
  EXPECT_DOUBLE_EQ(n->get_parameter("control.rate").as_double(), 100.0);
  EXPECT_EQ(n->get_parameter("legacy_b.cmd_id").as_int(), 0x701);
  EXPECT_EQ(n->count_publishers("/imu/data_raw"), 0u);
  rclcpp::NodeOptions bad;
  bad.append_parameter_override("dry_run", true);
  bad.append_parameter_override("build", "b");
  bad.append_parameter_override("protocol.type", "canopen");
  EXPECT_THROW(std::make_shared<luchs_driver::LuchsDriverNode>(bad), std::invalid_argument);
}

TEST(Parse, BuildAndProtocol)
{
  EXPECT_EQ(luchs_driver::parse_build("b"), luchs_sdk::Build::kLuchsB);
  EXPECT_EQ(luchs_driver::parse_protocol("", luchs_sdk::Build::kLuchsB), luchs_sdk::Protocol::kLci);
  EXPECT_EQ(luchs_driver::parse_protocol("", luchs_sdk::Build::kFourWheelSteer),
    luchs_sdk::Protocol::kLegacyA);
  EXPECT_EQ(luchs_driver::parse_protocol("legacy_b", luchs_sdk::Build::kLuchsB),
    luchs_sdk::Protocol::kLegacyB);
}
