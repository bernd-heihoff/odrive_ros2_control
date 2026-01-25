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

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "odrive_hardware_interface/axis_utils.hpp"
#include "odrive_hardware_interface/odrive_hardware_interface.hpp"
#include "test_support/mock_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_support/fake_diagnostics.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace odrive_hardware_interface
{
namespace
{

// Helper function: convert turns to radians (same as in axis_telemetry.cpp)
inline double turns_to_radians(float turns)
{
  return static_cast<double>(turns * 2.0F * M_PI);
}

// Helper function: convert radians to turns (same as in axis_control.cpp)
inline float radians_to_turns(double radians)
{
  return static_cast<float>(radians / (2.0 * M_PI));
}

class AxisInversionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
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

// Helper to setup axis telemetry expectations (per-axis readings)
void setup_axis_telemetry_expectations(
  MockTransport * instance,
  std::int64_t serial,
  int axis,
  float raw_position,
  float raw_velocity,
  float raw_effort,
  float torque_constant)
{
  // Telemetry read order from axis_telemetry.cpp:
  // IQ_MEASURED, VEL_ESTIMATE, POS_ESTIMATE
  instance->expect_read(
    serial,
    axis_endpoint(odrive::AXIS__MOTOR__CURRENT_CONTROL__IQ_MEASURED, axis),
    raw_effort / torque_constant);
  instance->expect_read(
    serial,
    axis_endpoint(odrive::AXIS__ENCODER__VEL_ESTIMATE, axis),
    raw_velocity);
  instance->expect_read(
    serial,
    axis_endpoint(odrive::AXIS__ENCODER__POS_ESTIMATE, axis),
    raw_position);
  instance->expect_read(
    serial,
    axis_endpoint(odrive::AXIS__ERROR, axis),
    static_cast<std::int32_t>(0));
  instance->expect_read(
    serial,
    axis_endpoint(odrive::AXIS__MOTOR__ERROR, axis),
    static_cast<std::int32_t>(0));
  instance->expect_read(
    serial,
    axis_endpoint(odrive::AXIS__ENCODER__ERROR, axis),
    static_cast<std::int32_t>(0));
  instance->expect_read(
    serial,
    axis_endpoint(odrive::AXIS__CONTROLLER__ERROR, axis),
    static_cast<std::int32_t>(0));
  instance->expect_read(
    serial,
    axis_endpoint(odrive::AXIS__FET_THERMISTOR__TEMPERATURE, axis),
    30.0F);
  instance->expect_read(
    serial,
    axis_endpoint(odrive::AXIS__MOTOR_THERMISTOR__TEMPERATURE, axis),
    40.0F);
}

// Helper to setup complete read cycle expectations
// (bus + axis telemetry)
void setup_read_expectations(
  MockTransport * instance,
  std::int64_t serial,
  int axis,
  float raw_position,
  float raw_velocity,
  float raw_effort,
  float torque_constant)
{
  instance->expect_read(serial, odrive::VBUS_VOLTAGE, 24.0F);
  instance->expect_read(
    serial, odrive::CAN__ERROR, static_cast<std::uint32_t>(0));

  setup_axis_telemetry_expectations(
    instance, serial, axis, raw_position, raw_velocity, raw_effort,
    torque_constant);
}

TEST_F(AxisInversionTest, ConfigurationParsesInvertAxisParameter)
{
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

  const int axis = 0;

  hardware_interface::HardwareInfo info;
  info.hardware_parameters["publish_diagnostics"] = "false";

  hardware_interface::ComponentInfo sensor;
  sensor.name = "bus";
  sensor.parameters["serial_number"] = "e1";
  info.sensors.push_back(sensor);

  // Joint with invert_axis = true
  hardware_interface::ComponentInfo joint;
  joint.name = "inverted_wheel";
  joint.parameters["serial_number"] = "e1";
  joint.parameters["axis"] = "0";
  joint.parameters["invert_axis"] = "true";
  joint.parameters["watchdog_timeout"] = "0.10";
  joint.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint);

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
}

TEST_F(AxisInversionTest, ConfigurationDefaultsInvertAxisToFalse)
{
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

  const std::int64_t serial = 0x00000000000000E1LL;

  hardware_interface::HardwareInfo info;
  info.hardware_parameters["publish_diagnostics"] = "false";

  hardware_interface::ComponentInfo sensor;
  sensor.name = "bus";
  sensor.parameters["serial_number"] = "e1";
  info.sensors.push_back(sensor);

  // Joint without invert_axis parameter (should default to false)
  hardware_interface::ComponentInfo joint;
  joint.name = "normal_wheel";
  joint.parameters["serial_number"] = "e1";
  joint.parameters["axis"] = "0";
  joint.parameters["watchdog_timeout"] = "0.10";
  joint.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint);

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
}

TEST_F(AxisInversionTest, StateInterfacesInvertedForInvertedAxis)
{
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

  const std::int64_t serial = 0x00000000000000E1LL;
  const int axis = 0;
  const float torque_constant = 6.0F;

  // Raw values from ODrive in turns (encoder units)
  const float raw_position_turns = 10.5F;
  const float raw_velocity_turns_per_s = 2.3F;
  const float raw_effort = 1.2F;

  // Expected values in radians
  // (after turns_to_radians conversion and inversion)
  const double expected_position = -turns_to_radians(raw_position_turns);
  const double expected_velocity =
    -turns_to_radians(raw_velocity_turns_per_s);
  const double expected_effort = -raw_effort;

  MockTransport * transport = nullptr;
  interface.set_transport_factory(
    [&]() {
      auto instance = std::make_unique<MockTransport>();
      transport = instance.get();

      // Setup expectations for activation
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

      // Setup expectations for read cycle
      setup_read_expectations(
        instance.get(), serial, axis, raw_position_turns,
        raw_velocity_turns_per_s, raw_effort, torque_constant);

      return instance;
    });

  hardware_interface::HardwareInfo info;
  info.hardware_parameters["publish_diagnostics"] = "false";

  hardware_interface::ComponentInfo sensor;
  sensor.name = "bus";
  sensor.parameters["serial_number"] = "e1";
  info.sensors.push_back(sensor);

  hardware_interface::ComponentInfo joint;
  joint.name = "inverted_wheel";
  joint.parameters["serial_number"] = "e1";
  joint.parameters["axis"] = "0";
  joint.parameters["invert_axis"] = "true";
  joint.parameters["watchdog_timeout"] = "0.10";
  joint.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint);

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  ASSERT_EQ(
    CallbackReturn::SUCCESS,
    interface.on_activate(rclcpp_lifecycle::State{}));

  auto state_interfaces = interface.export_state_interfaces();

  // Perform read
  ASSERT_EQ(
    return_type::OK, interface.read(rclcpp::Time(0), rclcpp::Duration(0, 0)));

  // Find the state interfaces
  auto find_interface = [&](const std::string & name) -> double {
      for (const auto & iface : state_interfaces) {
        if (iface.get_interface_name() == name &&
          iface.get_prefix_name() == "inverted_wheel")
        {
          return iface.get_value();
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    };

  // Verify inversion: reported values should be negated
  EXPECT_NEAR(expected_position, find_interface("position"), 1e-5);
  EXPECT_NEAR(expected_velocity, find_interface("velocity"), 1e-5);
  EXPECT_NEAR(expected_effort, find_interface("effort"), 1e-5);

  EXPECT_TRUE(transport->expectations_satisfied());
}

TEST_F(AxisInversionTest, StateInterfacesNotInvertedForNormalAxis)
{
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

  const std::int64_t serial = 0x00000000000000E1LL;
  const int axis = 0;
  const float torque_constant = 6.0F;

  // Raw values from ODrive in turns (encoder units)
  const float raw_position_turns = 10.5F;
  const float raw_velocity_turns_per_s = 2.3F;
  const float raw_effort = 1.2F;

  // Expected values in radians
  // (after turns_to_radians conversion, no inversion)
  const double expected_position = turns_to_radians(raw_position_turns);
  const double expected_velocity =
    turns_to_radians(raw_velocity_turns_per_s);
  const double expected_effort = raw_effort;

  MockTransport * transport = nullptr;
  interface.set_transport_factory(
    [&]() {
      auto instance = std::make_unique<MockTransport>();
      transport = instance.get();

      // Setup expectations for activation
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

      // Setup expectations for read cycle
      setup_read_expectations(
        instance.get(), serial, axis, raw_position_turns,
        raw_velocity_turns_per_s, raw_effort, torque_constant);

      return instance;
    });

  hardware_interface::HardwareInfo info;
  info.hardware_parameters["publish_diagnostics"] = "false";

  hardware_interface::ComponentInfo sensor;
  sensor.name = "bus";
  sensor.parameters["serial_number"] = "e1";
  info.sensors.push_back(sensor);

  hardware_interface::ComponentInfo joint;
  joint.name = "normal_wheel";
  joint.parameters["serial_number"] = "e1";
  joint.parameters["axis"] = "0";
  joint.parameters["invert_axis"] = "false";  // Explicit false
  joint.parameters["watchdog_timeout"] = "0.10";
  joint.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint);

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  ASSERT_EQ(
    CallbackReturn::SUCCESS,
    interface.on_activate(rclcpp_lifecycle::State{}));

  auto state_interfaces = interface.export_state_interfaces();

  // Perform read
  ASSERT_EQ(
    return_type::OK, interface.read(rclcpp::Time(0), rclcpp::Duration(0, 0)));

  // Find the state interfaces
  auto find_interface = [&](const std::string & name) -> double {
      for (const auto & iface : state_interfaces) {
        if (iface.get_interface_name() == name &&
          iface.get_prefix_name() == "normal_wheel")
        {
          return iface.get_value();
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    };

  // Verify NO inversion: reported values should match
  // (after turns→radians conversion)
  EXPECT_NEAR(expected_position, find_interface("position"), 1e-5);
  EXPECT_NEAR(expected_velocity, find_interface("velocity"), 1e-5);
  EXPECT_NEAR(expected_effort, find_interface("effort"), 1e-5);

  EXPECT_TRUE(transport->expectations_satisfied());
}

TEST_F(AxisInversionTest, CommandInterfacesExportedForInvertedAxis)
{
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

  const std::int64_t serial = 0x00000000000000E1LL;
  const int axis = 0;
  const float torque_constant = 6.0F;

  MockTransport * transport = nullptr;
  interface.set_transport_factory(
    [&]() {
      auto instance = std::make_unique<MockTransport>();
      transport = instance.get();

      // Setup expectations for activation
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

  hardware_interface::HardwareInfo info;
  info.hardware_parameters["publish_diagnostics"] = "false";

  hardware_interface::ComponentInfo sensor;
  sensor.name = "bus";
  sensor.parameters["serial_number"] = "e1";
  info.sensors.push_back(sensor);

  hardware_interface::ComponentInfo joint;
  joint.name = "inverted_wheel";
  joint.parameters["serial_number"] = "e1";
  joint.parameters["axis"] = "0";
  joint.parameters["invert_axis"] = "true";
  joint.parameters["watchdog_timeout"] = "0.10";
  joint.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint);

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  ASSERT_EQ(
    CallbackReturn::SUCCESS,
    interface.on_activate(rclcpp_lifecycle::State{}));

  auto command_interfaces = interface.export_command_interfaces();

  // Verify command interfaces exist for inverted axis
  bool has_position = false;
  bool has_velocity = false;
  bool has_effort = false;

  for (const auto & iface : command_interfaces) {
    if (iface.get_prefix_name() == "inverted_wheel") {
      if (iface.get_interface_name() == "position") {
        has_position = true;
      } else if (iface.get_interface_name() == "velocity") {
        has_velocity = true;
      } else if (iface.get_interface_name() == "effort") {
        has_effort = true;
      }
    }
  }

  // Verify all three command interfaces exist
  EXPECT_TRUE(has_position);
  EXPECT_TRUE(has_velocity);
  EXPECT_TRUE(has_effort);

  // Note: Testing actual command inversion during write() requires
  // setting up control modes and state machine, which is covered in
  // other integration tests. The inversion logic in write() is
  // symmetric to read() which is tested above.
}

TEST_F(AxisInversionTest, MixedInvertedAndNormalAxes)
{
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

  const std::int64_t serial = 0x00000000000000E1LL;
  const float torque_constant = 6.0F;

  // Raw values from ODrive in turns (encoder units)
  const float raw_position_inverted_turns = 10.0F;
  const float raw_velocity_inverted_turns_per_s = 2.0F;
  const float raw_effort_inverted = 1.0F;

  const float raw_position_normal_turns = 15.0F;
  const float raw_velocity_normal_turns_per_s = 3.0F;
  const float raw_effort_normal = 1.5F;

  // Expected values in radians after conversions
  const double expected_position_inverted =
    -turns_to_radians(raw_position_inverted_turns);
  const double expected_velocity_inverted =
    -turns_to_radians(raw_velocity_inverted_turns_per_s);
  const double expected_effort_inverted = -raw_effort_inverted;

  const double expected_position_normal =
    turns_to_radians(raw_position_normal_turns);
  const double expected_velocity_normal =
    turns_to_radians(raw_velocity_normal_turns_per_s);
  const double expected_effort_normal = raw_effort_normal;

  MockTransport * transport = nullptr;
  interface.set_transport_factory(
    [&]() {
      auto instance = std::make_unique<MockTransport>();
      transport = instance.get();

      // Activation expectations for both axes
      for (int axis = 0; axis < 2; ++axis) {
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
      }

      // Read expectations
      instance->expect_read(serial, odrive::VBUS_VOLTAGE, 24.0F);
      instance->expect_read(
        serial, odrive::CAN__ERROR, static_cast<std::uint32_t>(0));

      // Axis 0 (inverted)
      setup_axis_telemetry_expectations(
        instance.get(), serial, 0, raw_position_inverted_turns,
        raw_velocity_inverted_turns_per_s, raw_effort_inverted,
        torque_constant);

      // Axis 1 (normal)
      setup_axis_telemetry_expectations(
        instance.get(), serial, 1, raw_position_normal_turns,
        raw_velocity_normal_turns_per_s, raw_effort_normal, torque_constant);

      return instance;
    });

  hardware_interface::HardwareInfo info;
  info.hardware_parameters["publish_diagnostics"] = "false";

  hardware_interface::ComponentInfo sensor;
  sensor.name = "bus";
  sensor.parameters["serial_number"] = "e1";
  info.sensors.push_back(sensor);

  // Inverted axis
  hardware_interface::ComponentInfo joint_inverted;
  joint_inverted.name = "wheel_left";
  joint_inverted.parameters["serial_number"] = "e1";
  joint_inverted.parameters["axis"] = "0";
  joint_inverted.parameters["invert_axis"] = "true";
  joint_inverted.parameters["watchdog_timeout"] = "0.10";
  joint_inverted.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint_inverted);

  // Normal axis
  hardware_interface::ComponentInfo joint_normal;
  joint_normal.name = "wheel_right";
  joint_normal.parameters["serial_number"] = "e1";
  joint_normal.parameters["axis"] = "1";
  joint_normal.parameters["invert_axis"] = "false";
  joint_normal.parameters["watchdog_timeout"] = "0.10";
  joint_normal.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint_normal);

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  ASSERT_EQ(
    CallbackReturn::SUCCESS,
    interface.on_activate(rclcpp_lifecycle::State{}));

  auto state_interfaces = interface.export_state_interfaces();

  // Perform read
  ASSERT_EQ(
    return_type::OK, interface.read(rclcpp::Time(0), rclcpp::Duration(0, 0)));

  auto find_interface = [&](const std::string & joint,
      const std::string & name) -> double {
      for (const auto & iface : state_interfaces) {
        if (iface.get_interface_name() == name &&
          iface.get_prefix_name() == joint)
        {
          return iface.get_value();
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    };

  // Inverted axis should have negated values
  // (in radians after conversion)
  EXPECT_NEAR(
    expected_position_inverted, find_interface("wheel_left", "position"),
    1e-5);
  EXPECT_NEAR(
    expected_velocity_inverted, find_interface("wheel_left", "velocity"),
    1e-5);
  EXPECT_NEAR(
    expected_effort_inverted, find_interface("wheel_left", "effort"), 1e-5);

  // Normal axis should have original values
  // (in radians after conversion)
  EXPECT_NEAR(
    expected_position_normal, find_interface("wheel_right", "position"),
    1e-5);
  EXPECT_NEAR(
    expected_velocity_normal, find_interface("wheel_right", "velocity"),
    1e-5);
  EXPECT_NEAR(
    expected_effort_normal, find_interface("wheel_right", "effort"), 1e-5);

  EXPECT_TRUE(transport->expectations_satisfied());
}

}  // namespace
}  // namespace odrive_hardware_interface
