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
#include "odrive_hardware_interface/axis_control.hpp"
#include "odrive_hardware_interface/axis_utils.hpp"
#include "odrive_hardware_interface/odrive_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_support/fake_diagnostics.hpp"
#include "test_support/mock_transport.hpp"

namespace odrive_hardware_interface
{
namespace
{
class SafetyGatingTest : public ::testing::Test
{
};

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

TEST_F(SafetyGatingTest, WriteIsSkippedWhenInactiveEvenIfCommandModeSet)
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
      return instance;
    });

  auto info = make_minimal_info("a1");
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());

  auto command_interfaces = interface.export_command_interfaces();
  for (auto & command : command_interfaces) {
    if (command.get_prefix_name() == "wheel") {
      command.set_value(0.0);
    }
  }

  EXPECT_EQ(
    return_type::OK,
    interface.prepare_command_mode_switch(
      {"wheel/" + std::string(hardware_interface::HW_IF_EFFORT)},
      {}));

  // Not activated: outputs must be gated.
  EXPECT_EQ(return_type::OK, interface.write(rclcpp::Time{}, rclcpp::Duration(0, 0)));
  EXPECT_TRUE(transport->expectations_satisfied());
}

TEST_F(SafetyGatingTest, FaultedAxisIsMaskedAndIdled)
{
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

  const std::int64_t serial = 0x00000000000000B1LL;
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

  auto info = make_minimal_info("b1");
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());

  transport->expect_call(serial, axis_endpoint(odrive::AXIS__CLEAR_ERRORS, axis));
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.on_activate(rclcpp_lifecycle::State{}));
  EXPECT_TRUE(transport->expectations_satisfied());

  // Provide a telemetry sample indicating an axis fault.
  const float vbus_voltage = 24.0F;
  const int32_t odrive_error = 0;
  const float iq_measured = 0.0F;
  const float vel_estimate = 0.0F;
  const float pos_estimate = 0.0F;
  const int32_t axis_error = 0x00000001;
  const int32_t motor_error = 0;
  const int32_t encoder_error = 0;
  const int32_t controller_error = 0;
  const float fet_temperature = 30.0F;
  const float motor_temperature = 30.0F;

  transport->expect_read(serial, odrive::VBUS_VOLTAGE, vbus_voltage);
  transport->expect_read(serial, 0, odrive_error);
  transport->expect_read(
    serial,
    axis_endpoint(odrive::AXIS__MOTOR__CURRENT_CONTROL__IQ_MEASURED, axis),
    iq_measured);
  transport->expect_read(
    serial,
    axis_endpoint(odrive::AXIS__ENCODER__VEL_ESTIMATE, axis),
    vel_estimate);
  transport->expect_read(
    serial,
    axis_endpoint(odrive::AXIS__ENCODER__POS_ESTIMATE, axis),
    pos_estimate);
  transport->expect_read(serial, axis_endpoint(odrive::AXIS__ERROR, axis), axis_error);
  transport->expect_read(serial, axis_endpoint(odrive::AXIS__MOTOR__ERROR, axis), motor_error);
  transport->expect_read(
    serial,
    axis_endpoint(odrive::AXIS__ENCODER__ERROR, axis),
    encoder_error);
  transport->expect_read(
    serial,
    axis_endpoint(odrive::AXIS__CONTROLLER__ERROR, axis),
    controller_error);
  transport->expect_read(
    serial,
    axis_endpoint(odrive::AXIS__FET_THERMISTOR__TEMPERATURE, axis),
    fet_temperature);
  transport->expect_read(
    serial,
    axis_endpoint(odrive::AXIS__MOTOR_THERMISTOR__TEMPERATURE, axis),
    motor_temperature);

  EXPECT_EQ(return_type::OK, interface.read(rclcpp::Time{}, rclcpp::Duration(0, 0)));
  EXPECT_TRUE(transport->expectations_satisfied());

  auto command_interfaces = interface.export_command_interfaces();
  for (auto & command : command_interfaces) {
    if (command.get_prefix_name() == "wheel") {
      command.set_value(0.0);
    }
  }

  EXPECT_EQ(
    return_type::OK,
    interface.prepare_command_mode_switch(
      {"wheel/" + std::string(hardware_interface::HW_IF_EFFORT)},
      {}));

  // Masking should idle the faulted axis and skip the torque write.
  transport->expect_write(
    serial,
    axis_endpoint(odrive::AXIS__REQUESTED_STATE, axis),
    static_cast<std::int32_t>(kAxisStateIdle));

  EXPECT_EQ(return_type::OK, interface.write(rclcpp::Time{}, rclcpp::Duration(0, 0)));
  EXPECT_TRUE(transport->expectations_satisfied());
}

}  // namespace
}  // namespace odrive_hardware_interface
