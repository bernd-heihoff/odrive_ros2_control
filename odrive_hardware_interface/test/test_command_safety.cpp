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
#include <cstdint>
#include <limits>
#include <string>

#include "hardware_interface/hardware_info.hpp"
#include "odrive_hardware_interface/axis_utils.hpp"
#include "odrive_hardware_interface/odrive_hardware_interface.hpp"
#include "test_support/mock_transport.hpp"
#include "rclcpp/rclcpp.hpp"

namespace odrive_hardware_interface
{
namespace
{
TEST(CommandSafetyTest, RejectsNanCommands)
{
  ODriveHardwareInterface interface;
  const std::int64_t serial = 0x00000000000000A1LL;
  const int axis = 0;
  const float torque_constant = 2.0F;
  MockTransport * transport = nullptr;

  interface.set_transport_factory(
    [&]() {
      auto instance = std::make_unique<MockTransport>();
      transport = instance.get();
      instance->expect_read(
        serial,
        axis_endpoint(odrive::AXIS__MOTOR__CONFIG__TORQUE_CONSTANT, axis),
        torque_constant);
      instance->expect_write(
        serial,
        axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, axis),
        static_cast<bool>(false));
      return instance;
    });

  hardware_interface::HardwareInfo info;

  hardware_interface::ComponentInfo sensor;
  sensor.name = "bus";
  sensor.parameters["serial_number"] = "a1";
  info.sensors.push_back(sensor);

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = "a1";
  joint.parameters["axis"] = "0";
  joint.parameters["watchdog_timeout"] = "0.10";
  joint.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint);

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.on_init(info));
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());

  auto command_interfaces = interface.export_command_interfaces();
  auto set_command = [&](const std::string & interface_name, double value) {
      for (auto & command : command_interfaces) {
        if (command.get_prefix_name() == "wheel" &&
          command.get_interface_name() == interface_name)
        {
          command.set_value(value);
          return;
        }
      }
      ADD_FAILURE() << "Failed to locate command interface wheel/" << interface_name;
    };

  set_command(hardware_interface::HW_IF_POSITION, std::numeric_limits<double>::quiet_NaN());
  set_command(hardware_interface::HW_IF_VELOCITY, 0.0);
  set_command(hardware_interface::HW_IF_EFFORT, 0.0);

  EXPECT_EQ(
    return_type::OK,
    interface.prepare_command_mode_switch(
      {"wheel/" + std::string(hardware_interface::HW_IF_EFFORT)},
      {}));
  EXPECT_EQ(
    return_type::OK,
    interface.prepare_command_mode_switch(
      {"wheel/" + std::string(hardware_interface::HW_IF_VELOCITY)},
      {}));
  EXPECT_EQ(
    return_type::OK,
    interface.prepare_command_mode_switch(
      {"wheel/" + std::string(hardware_interface::HW_IF_POSITION)},
      {}));

  EXPECT_EQ(return_type::ERROR, interface.write(rclcpp::Time{}, rclcpp::Duration(0, 0)));
  EXPECT_TRUE(transport->expectations_satisfied());
}

TEST(CommandSafetyTest, RejectsCommandsOutsideLimits)
{
  ODriveHardwareInterface interface;
  const std::int64_t serial = 0x00000000000000B1LL;
  const int axis = 1;
  const float torque_constant = 3.0F;
  MockTransport * transport = nullptr;

  const double position_max = 1.0;

  interface.set_transport_factory(
    [&]() {
      auto instance = std::make_unique<MockTransport>();
      transport = instance.get();
      instance->expect_read(
        serial,
        axis_endpoint(odrive::AXIS__MOTOR__CONFIG__TORQUE_CONSTANT, axis),
        torque_constant);
      instance->expect_write(
        serial,
        axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, axis),
        static_cast<bool>(false));
      return instance;
    });

  hardware_interface::HardwareInfo info;

  hardware_interface::ComponentInfo sensor;
  sensor.name = "bus";
  sensor.parameters["serial_number"] = "b1";
  info.sensors.push_back(sensor);

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = "b1";
  joint.parameters["axis"] = "1";
  joint.parameters["watchdog_timeout"] = "0.10";
  joint.parameters["enable_watchdog"] = "false";
  joint.parameters["command_position_max"] = std::to_string(position_max);
  joint.parameters["enforce_command_limits"] = "true";
  info.joints.push_back(joint);

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.on_init(info));
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());

  auto command_interfaces = interface.export_command_interfaces();
  auto set_command = [&](const std::string & interface_name, double value) {
      for (auto & command : command_interfaces) {
        if (command.get_prefix_name() == "wheel" &&
          command.get_interface_name() == interface_name)
        {
          command.set_value(value);
          return;
        }
      }
      ADD_FAILURE() << "Failed to locate command interface wheel/" << interface_name;
    };

  set_command(hardware_interface::HW_IF_POSITION, position_max + 0.1);
  set_command(hardware_interface::HW_IF_VELOCITY, 0.0);
  set_command(hardware_interface::HW_IF_EFFORT, 0.0);

  EXPECT_EQ(
    return_type::OK,
    interface.prepare_command_mode_switch(
      {"wheel/" + std::string(hardware_interface::HW_IF_EFFORT)},
      {}));
  EXPECT_EQ(
    return_type::OK,
    interface.prepare_command_mode_switch(
      {"wheel/" + std::string(hardware_interface::HW_IF_VELOCITY)},
      {}));
  EXPECT_EQ(
    return_type::OK,
    interface.prepare_command_mode_switch(
      {"wheel/" + std::string(hardware_interface::HW_IF_POSITION)},
      {}));

  EXPECT_EQ(return_type::ERROR, interface.write(rclcpp::Time{}, rclcpp::Duration(0, 0)));
  EXPECT_TRUE(transport->expectations_satisfied());
}

TEST(CommandSafetyTest, AcceptsCommandsWithinLimits)
{
  ODriveHardwareInterface interface;
  const std::int64_t serial = 0x00000000000000C1LL;
  const int axis = 0;
  const float torque_constant = 4.0F;
  MockTransport * transport = nullptr;

  interface.set_transport_factory(
    [&]() {
      auto instance = std::make_unique<MockTransport>();
      transport = instance.get();
      instance->expect_read(
        serial,
        axis_endpoint(odrive::AXIS__MOTOR__CONFIG__TORQUE_CONSTANT, axis),
        torque_constant);
      instance->expect_write(
        serial,
        axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, axis),
        static_cast<bool>(false));
      return instance;
    });

  hardware_interface::HardwareInfo info;

  hardware_interface::ComponentInfo sensor;
  sensor.name = "bus";
  sensor.parameters["serial_number"] = "c1";
  info.sensors.push_back(sensor);

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = "c1";
  joint.parameters["axis"] = "0";
  joint.parameters["watchdog_timeout"] = "0.10";
  joint.parameters["enable_watchdog"] = "false";
  joint.parameters["command_position_min"] = "-1.0";
  joint.parameters["command_position_max"] = "1.0";
  joint.parameters["command_velocity_max"] = "5.0";
  joint.parameters["command_effort_max"] = "10.0";
  info.joints.push_back(joint);

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.on_init(info));
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());

  auto command_interfaces = interface.export_command_interfaces();
  auto set_command = [&](const std::string & interface_name, double value) {
      for (auto & command : command_interfaces) {
        if (command.get_prefix_name() == "wheel" &&
          command.get_interface_name() == interface_name)
        {
          command.set_value(value);
          return;
        }
      }
      ADD_FAILURE() << "Failed to locate command interface wheel/" << interface_name;
    };

  const double position = 0.5;
  const double velocity = 1.0;
  const double effort = 2.0;

  set_command(hardware_interface::HW_IF_POSITION, position);
  set_command(hardware_interface::HW_IF_VELOCITY, velocity);
  set_command(hardware_interface::HW_IF_EFFORT, effort);

  EXPECT_EQ(
    return_type::OK,
    interface.prepare_command_mode_switch(
      {"wheel/" + std::string(hardware_interface::HW_IF_EFFORT)},
      {}));
  EXPECT_EQ(
    return_type::OK,
    interface.prepare_command_mode_switch(
      {"wheel/" + std::string(hardware_interface::HW_IF_VELOCITY)},
      {}));
  EXPECT_EQ(
    return_type::OK,
    interface.prepare_command_mode_switch(
      {"wheel/" + std::string(hardware_interface::HW_IF_POSITION)},
      {}));

  const auto position_endpoint = axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_POS, axis);
  const auto velocity_endpoint = axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_VEL, axis);
  const auto torque_endpoint = axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_TORQUE, axis);

  transport->expect_write(
    serial,
    position_endpoint,
    static_cast<float>(position / (2.0 * M_PI)));
  transport->expect_write(
    serial,
    velocity_endpoint,
    static_cast<float>(velocity / (2.0 * M_PI)));
  transport->expect_write(
    serial,
    torque_endpoint,
    static_cast<float>(effort));

  EXPECT_EQ(return_type::OK, interface.write(rclcpp::Time{}, rclcpp::Duration(0, 0)));
  EXPECT_TRUE(transport->expectations_satisfied());
}
}  // namespace
}  // namespace odrive_hardware_interface
