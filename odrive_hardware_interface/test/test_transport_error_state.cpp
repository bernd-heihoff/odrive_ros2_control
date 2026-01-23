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
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "odrive_hardware_interface/axis_utils.hpp"
#include "odrive_hardware_interface/odrive_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_support/fake_diagnostics.hpp"
#include "test_support/mock_transport.hpp"

namespace odrive_hardware_interface
{
namespace
{
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

hardware_interface::HardwareInfo make_minimal_info_no_sensors(const std::string & serial_hex)
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["publish_diagnostics"] = "false";

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = serial_hex;
  joint.parameters["axis"] = "0";
  joint.parameters["watchdog_timeout"] = "0.10";
  joint.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint);

  return info;
}

hardware_interface::StateInterface * find_state_interface(
  std::vector<hardware_interface::StateInterface> & state_interfaces,
  const std::string & prefix,
  const std::string & interface_name)
{
  for (auto & state : state_interfaces) {
    if (state.get_prefix_name() == prefix && state.get_interface_name() == interface_name) {
      return &state;
    }
  }
  return nullptr;
}

TEST(TransportErrorStateTest, ExportsTransportErrorStateInterface)
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

  auto state_interfaces = interface.export_state_interfaces();
  auto * transport_error = find_state_interface(state_interfaces, "bus", "transport_error");
  ASSERT_NE(nullptr, transport_error);
  EXPECT_EQ(0.0, transport_error->get_value());
}

TEST(TransportErrorStateTest, ExportsJointIoStateInterfaces)
{
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

  const std::int64_t serial = 0x00000000000000D1LL;
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

  auto info = make_minimal_info_no_sensors("d1");
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());

  auto state_interfaces = interface.export_state_interfaces();
  auto * read_error = find_state_interface(state_interfaces, "wheel", "read_error");
  auto * write_error = find_state_interface(state_interfaces, "wheel", "write_error");
  auto * telemetry_valid = find_state_interface(state_interfaces, "wheel", "telemetry_valid");
  auto * healthy = find_state_interface(state_interfaces, "wheel", "healthy");
  ASSERT_NE(nullptr, read_error);
  ASSERT_NE(nullptr, write_error);
  ASSERT_NE(nullptr, telemetry_valid);
  ASSERT_NE(nullptr, healthy);
  EXPECT_EQ(0.0, read_error->get_value());
  EXPECT_EQ(0.0, write_error->get_value());
  EXPECT_EQ(0.0, telemetry_valid->get_value());
  EXPECT_EQ(0.0, healthy->get_value());
}

TEST(TransportErrorStateTest, HealthyResetsToUnhealthyOnReadFailure)
{
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

  const std::int64_t serial = 0x00000000000000F1LL;
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

  auto info = make_minimal_info_no_sensors("f1");
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());

  auto state_interfaces = interface.export_state_interfaces();
  auto * healthy = find_state_interface(state_interfaces, "wheel", "healthy");
  ASSERT_NE(nullptr, healthy);
  EXPECT_EQ(0.0, healthy->get_value());

  // First read succeeds with all errors clear => healthy.
  {
    const std::uint32_t can_error = 0U;
    transport->expect_read(serial, odrive::CAN__ERROR, can_error);

    const float iq_measured = 0.0F;
    transport->expect_read(
      serial,
      axis_endpoint(odrive::AXIS__MOTOR__CURRENT_CONTROL__IQ_MEASURED, axis),
      iq_measured);

    const float vel_estimate = 0.0F;
    transport->expect_read(
      serial,
      axis_endpoint(odrive::AXIS__ENCODER__VEL_ESTIMATE, axis),
      vel_estimate);

    const float pos_estimate = 0.0F;
    transport->expect_read(
      serial,
      axis_endpoint(odrive::AXIS__ENCODER__POS_ESTIMATE, axis),
      pos_estimate);

    const std::int32_t axis_error = 0;
    transport->expect_read(serial, axis_endpoint(odrive::AXIS__ERROR, axis), axis_error);
    const std::int32_t motor_error = 0;
    transport->expect_read(serial, axis_endpoint(odrive::AXIS__MOTOR__ERROR, axis), motor_error);
    const std::int32_t encoder_error = 0;
    transport->expect_read(
      serial, axis_endpoint(odrive::AXIS__ENCODER__ERROR, axis), encoder_error);
    const std::int32_t controller_error = 0;
    transport->expect_read(
      serial, axis_endpoint(odrive::AXIS__CONTROLLER__ERROR, axis), controller_error);

    const float fet_temperature = 20.0F;
    transport->expect_read(
      serial,
      axis_endpoint(odrive::AXIS__FET_THERMISTOR__TEMPERATURE, axis),
      fet_temperature);
    const float motor_temperature = 20.0F;
    transport->expect_read(
      serial,
      axis_endpoint(odrive::AXIS__MOTOR_THERMISTOR__TEMPERATURE, axis),
      motor_temperature);

    EXPECT_EQ(return_type::OK, interface.read(rclcpp::Time{}, rclcpp::Duration(0, 0)));
    EXPECT_TRUE(transport->expectations_satisfied());
    EXPECT_EQ(1.0, healthy->get_value());
  }

  // Second read fails early during telemetry => healthy must reset to unhealthy.
  {
    const std::uint32_t can_error = 0U;
    transport->expect_read(serial, odrive::CAN__ERROR, can_error);

    const float iq_measured = 0.0F;
    transport->expect_read(
      serial,
      axis_endpoint(odrive::AXIS__MOTOR__CURRENT_CONTROL__IQ_MEASURED, axis),
      iq_measured,
      -11);

    EXPECT_EQ(return_type::ERROR, interface.read(rclcpp::Time{}, rclcpp::Duration(0, 0)));
    EXPECT_TRUE(transport->expectations_satisfied());
    EXPECT_EQ(0.0, healthy->get_value());
  }
}

TEST(TransportErrorStateTest, UpdatesOnVbusReadFailure)
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

  auto state_interfaces = interface.export_state_interfaces();
  auto * transport_error = find_state_interface(state_interfaces, "bus", "transport_error");
  ASSERT_NE(nullptr, transport_error);

  const float vbus_voltage = 0.0F;
  transport->expect_read(serial, odrive::VBUS_VOLTAGE, vbus_voltage, -5);

  EXPECT_EQ(return_type::ERROR, interface.read(rclcpp::Time{}, rclcpp::Duration(0, 0)));
  EXPECT_TRUE(transport->expectations_satisfied());
  EXPECT_EQ(-5.0, transport_error->get_value());
}

TEST(TransportErrorStateTest, UpdatesOnAxisTelemetryReadFailure)
{
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

  const std::int64_t serial = 0x00000000000000C1LL;
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

  auto info = make_minimal_info("c1");
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());

  auto state_interfaces = interface.export_state_interfaces();
  auto * transport_error = find_state_interface(state_interfaces, "bus", "transport_error");
  ASSERT_NE(nullptr, transport_error);

  const float vbus_voltage = 24.0F;
  transport->expect_read(serial, odrive::VBUS_VOLTAGE, vbus_voltage);

  const std::uint32_t can_error = 0U;
  transport->expect_read(serial, odrive::CAN__ERROR, can_error);

  // Fail the first telemetry read (motor current).
  const float iq_measured = 0.0F;
  transport->expect_read(
    serial,
    axis_endpoint(odrive::AXIS__MOTOR__CURRENT_CONTROL__IQ_MEASURED, axis),
    iq_measured,
    -7);

  EXPECT_EQ(return_type::ERROR, interface.read(rclcpp::Time{}, rclcpp::Duration(0, 0)));
  EXPECT_TRUE(transport->expectations_satisfied());
  EXPECT_EQ(-7.0, transport_error->get_value());
}

TEST(TransportErrorStateTest, UpdatesJointReadErrorWhenNoSensors)
{
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

  const std::int64_t serial = 0x00000000000000E1LL;
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

  auto info = make_minimal_info_no_sensors("e1");
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());

  auto state_interfaces = interface.export_state_interfaces();
  auto * read_error = find_state_interface(state_interfaces, "wheel", "read_error");
  auto * telemetry_valid = find_state_interface(state_interfaces, "wheel", "telemetry_valid");
  ASSERT_NE(nullptr, read_error);
  ASSERT_NE(nullptr, telemetry_valid);

  const std::uint32_t can_error = 0U;
  transport->expect_read(serial, odrive::CAN__ERROR, can_error);

  // Fail the first telemetry read (motor current).
  const float iq_measured = 0.0F;
  transport->expect_read(
    serial,
    axis_endpoint(odrive::AXIS__MOTOR__CURRENT_CONTROL__IQ_MEASURED, axis),
    iq_measured,
    -11);

  EXPECT_EQ(return_type::ERROR, interface.read(rclcpp::Time{}, rclcpp::Duration(0, 0)));
  EXPECT_TRUE(transport->expectations_satisfied());
  EXPECT_EQ(-11.0, read_error->get_value());
  EXPECT_EQ(0.0, telemetry_valid->get_value());
}

}  // namespace
}  // namespace odrive_hardware_interface
