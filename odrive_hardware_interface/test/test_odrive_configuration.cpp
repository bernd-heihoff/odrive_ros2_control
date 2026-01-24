// Copyright 2025 Wackerbot
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <string>

#include "hardware_interface/hardware_info.hpp"
#include "odrive_hardware_interface/odrive_configuration.hpp"

namespace odrive_hardware_interface
{
namespace
{
TEST(ConfigurationTest, ParsesValidHardwareInfo)
{
  hardware_interface::HardwareInfo info;

  hardware_interface::ComponentInfo sensor;
  sensor.name = "vbus";
  sensor.parameters["serial_number"] = "1234abcd";
  info.sensors.push_back(sensor);

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = "0001ffff";
  joint.parameters["axis"] = "1";
  joint.parameters["watchdog_timeout"] = "0.25";
  joint.parameters["enable_watchdog"] = "true";
  info.joints.push_back(joint);

  HardwareConfiguration config;
  std::string error;
  ASSERT_TRUE(parse_hardware_configuration(info, config, error)) << error;

  ASSERT_EQ(1u, config.sensors.size());
  EXPECT_EQ("vbus", config.sensors.front().name);
  EXPECT_EQ(0x1234abcd, config.sensors.front().serial_number);

  ASSERT_EQ(1u, config.joints.size());
  const auto & parsed_joint = config.joints.front();
  EXPECT_EQ("wheel", parsed_joint.name);
  EXPECT_EQ(0x0001ffff, parsed_joint.serial_number);
  EXPECT_EQ(1, parsed_joint.axis);
  EXPECT_TRUE(parsed_joint.enable_watchdog);
  EXPECT_DOUBLE_EQ(0.25, parsed_joint.watchdog_timeout);
}

TEST(ConfigurationTest, ParsesCommandLimitsWhenProvided)
{
  hardware_interface::HardwareInfo info;

  hardware_interface::ComponentInfo sensor;
  sensor.name = "vbus";
  sensor.parameters["serial_number"] = "1111";
  info.sensors.push_back(sensor);

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = "0001";
  joint.parameters["axis"] = "2";
  joint.parameters["watchdog_timeout"] = "0.10";
  joint.parameters["enable_watchdog"] = "false";
  joint.parameters["command_position_min"] = "-1.5";
  joint.parameters["command_position_max"] = "1.5";
  joint.parameters["command_velocity_min"] = "-5.0";
  joint.parameters["command_velocity_max"] = "5.0";
  joint.parameters["command_effort_min"] = "-10.0";
  joint.parameters["command_effort_max"] = "10.0";
  info.joints.push_back(joint);

  HardwareConfiguration config;
  std::string error;
  ASSERT_TRUE(parse_hardware_configuration(info, config, error)) << error;

  ASSERT_EQ(1u, config.joints.size());
  const auto & parsed_joint = config.joints.front();
  ASSERT_TRUE(parsed_joint.command_limits.position_min.has_value());
  EXPECT_DOUBLE_EQ(-1.5, parsed_joint.command_limits.position_min.value());
  ASSERT_TRUE(parsed_joint.command_limits.position_max.has_value());
  EXPECT_DOUBLE_EQ(1.5, parsed_joint.command_limits.position_max.value());
  ASSERT_TRUE(parsed_joint.command_limits.velocity_min.has_value());
  EXPECT_DOUBLE_EQ(-5.0, parsed_joint.command_limits.velocity_min.value());
  ASSERT_TRUE(parsed_joint.command_limits.velocity_max.has_value());
  EXPECT_DOUBLE_EQ(5.0, parsed_joint.command_limits.velocity_max.value());
  ASSERT_TRUE(parsed_joint.command_limits.effort_min.has_value());
  EXPECT_DOUBLE_EQ(-10.0, parsed_joint.command_limits.effort_min.value());
  ASSERT_TRUE(parsed_joint.command_limits.effort_max.has_value());
  EXPECT_DOUBLE_EQ(10.0, parsed_joint.command_limits.effort_max.value());
}

TEST(ConfigurationTest, RejectsInvertedCommandRange)
{
  hardware_interface::HardwareInfo info;

  hardware_interface::ComponentInfo sensor;
  sensor.name = "vbus";
  sensor.parameters["serial_number"] = "1111";
  info.sensors.push_back(sensor);

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = "0001";
  joint.parameters["axis"] = "0";
  joint.parameters["watchdog_timeout"] = "0.10";
  joint.parameters["enable_watchdog"] = "false";
  joint.parameters["command_position_min"] = "2.0";
  joint.parameters["command_position_max"] = "-2.0";
  info.joints.push_back(joint);

  HardwareConfiguration config;
  std::string error;
  EXPECT_FALSE(parse_hardware_configuration(info, config, error));
  EXPECT_NE(std::string::npos, error.find("command_position_min"));
}

TEST(ConfigurationTest, ReportsMissingParameters)
{
  hardware_interface::HardwareInfo info;

  hardware_interface::ComponentInfo sensor;
  sensor.name = "vbus";
  sensor.parameters["serial_number"] = "1234";
  info.sensors.push_back(sensor);

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = "0001";
  joint.parameters["watchdog_timeout"] = "0.1";
  joint.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint);

  HardwareConfiguration config;
  std::string error;
  EXPECT_FALSE(parse_hardware_configuration(info, config, error));
  EXPECT_NE(std::string::npos, error.find("axis"));
}
}  // namespace
}  // namespace odrive_hardware_interface
