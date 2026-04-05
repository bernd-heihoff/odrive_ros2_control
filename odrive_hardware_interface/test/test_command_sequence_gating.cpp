// Copyright 2026 Wackerbot
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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "test_support/fake_diagnostics.hpp"
#include "test_support/mock_transport.hpp"

namespace odrive_hardware_interface
{
namespace
{
constexpr const char * kCommandSequenceInterface = "odrive_command_seq";

class TestHardwareInterface : public ODriveHardwareInterface
{
public:
  CallbackReturn configure(const hardware_interface::HardwareInfo & info)
  {
    info_ = info;
    return configure_from_info(info);
  }
};

hardware_interface::HardwareInfo make_minimal_info(const std::string & serial_hex)
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["publish_diagnostics"] = "false";
  info.hardware_parameters["gate_stale_commands_with_command_sequence"] = "true";

  hardware_interface::ComponentInfo sensor;
  sensor.name = "bus";
  sensor.parameters["serial_number"] = serial_hex;
  info.sensors.push_back(sensor);

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = serial_hex;
  joint.parameters["axis"] = "0";
  joint.parameters["watchdog_timeout"] = "0.10";
  joint.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint);

  return info;
}

TEST(CommandSequenceGatingTest, SkipsWriteWhenSequenceIsUnchanged)
{
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

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
      instance->expect_call(
        serial,
        axis_endpoint(odrive::AXIS__CLEAR_ERRORS, axis));

      return instance;
    });

  auto info = make_minimal_info("a1");
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.on_activate(rclcpp_lifecycle::State{}));
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

  set_command(hardware_interface::HW_IF_EFFORT, 1.0);
  set_command(hardware_interface::HW_IF_VELOCITY, 0.0);
  set_command(hardware_interface::HW_IF_POSITION, 0.0);
  set_command(kCommandSequenceInterface, 1.0);

  EXPECT_EQ(
    return_type::OK,
    interface.prepare_command_mode_switch(
      {"wheel/" + std::string(hardware_interface::HW_IF_EFFORT)},
      {}));

  // First write: new sequence => torque setpoint written.
  transport->expect_write(
    serial,
    axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_TORQUE, axis),
    1.0F);
  EXPECT_EQ(return_type::OK, interface.write(rclcpp::Time{}, rclcpp::Duration(0, 0)));
  EXPECT_TRUE(transport->expectations_satisfied());

  // Second write: same sequence => must skip writing.
  // (No expectations set; any write/call would fail the test.)
  set_command(hardware_interface::HW_IF_EFFORT, 2.0);
  set_command(kCommandSequenceInterface, 1.0);
  EXPECT_EQ(return_type::OK, interface.write(rclcpp::Time{}, rclcpp::Duration(0, 0)));
  EXPECT_TRUE(transport->expectations_satisfied());
}

}  // namespace
}  // namespace odrive_hardware_interface
