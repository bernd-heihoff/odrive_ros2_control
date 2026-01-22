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

#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "odrive_hardware_interface/axis_utils.hpp"
#include "odrive_hardware_interface/odrive_hardware_interface.hpp"
#include "test_support/mock_transport.hpp"
#include "test_support/fake_diagnostics.hpp"

namespace odrive_hardware_interface
{
class DiagnosticsConfigTestHelper : public ODriveHardwareInterface
{
public:
  bool configure(const hardware_interface::HardwareInfo & info)
  {
    return on_init(info) == CallbackReturn::SUCCESS;
  }

  bool diagnostics_enabled() const
  {
    return diagnostics_config_.enabled;
  }

  bool has_diagnostics_node() const
  {
    return static_cast<bool>(diagnostics_);
  }

  bool has_diagnostics_updater() const
  {
    return static_cast<bool>(diagnostics_);
  }

  double diagnostics_period_seconds() const
  {
    return std::chrono::duration<double>(diagnostics_period_).count();
  }

  double warn_threshold() const
  {
    return diagnostics_config_.warn_temperature_deg_c;
  }

  double error_threshold() const
  {
    return diagnostics_config_.error_temperature_deg_c;
  }
};

class DiagnosticsTestHelper : public ODriveHardwareInterface
{
public:
  DiagnosticsTestHelper()
  {
    diagnostics_config_.warn_temperature_deg_c = 70.0;
    diagnostics_config_.error_temperature_deg_c = 90.0;
  }

  double warn_threshold() const
  {
    return diagnostics_config_.warn_temperature_deg_c;
  }

  double error_threshold() const
  {
    return diagnostics_config_.error_temperature_deg_c;
  }

  auto & add_joint(const std::string & name, std::int64_t serial = 0x1, int axis = 0)
  {
    info_.joints.emplace_back();
    info_.joints.back().name = name;
    joints_.emplace_back();
    joints_.back().serial_number = serial;
    joints_.back().axis = axis;
    return joints_.back();
  }

  auto & add_drive(const std::string & label, std::int64_t serial = 0x1)
  {
    drives_.emplace_back();
    drives_.back().label = label;
    drives_.back().serial_number = serial;
    drives_.back().can_error = 0;
    drives_.back().can_error_read_error = 0.0;
    return drives_.back();
  }

  auto & add_sensor(const std::string & name, std::int64_t serial = 0x1)
  {
    info_.sensors.emplace_back();
    info_.sensors.back().name = name;
    sensors_.emplace_back();
    sensors_.back().serial_number = serial;
    return sensors_.back();
  }

  void populate_joint_status(
    diagnostic_updater::DiagnosticStatusWrapper & status,
    std::size_t index) const
  {
    populate_joint_diagnostics(status, index);
  }

  void populate_drive_status(
    diagnostic_updater::DiagnosticStatusWrapper & status,
    std::size_t index) const
  {
    populate_drive_diagnostics(status, index);
  }

  void populate_sensor_status(
    diagnostic_updater::DiagnosticStatusWrapper & status,
    std::size_t index) const
  {
    populate_sensor_diagnostics(status, index);
  }
};
}  // namespace odrive_hardware_interface

namespace
{
using odrive_hardware_interface::DiagnosticsConfigTestHelper;
using odrive_hardware_interface::DiagnosticsTestHelper;
using odrive_hardware_interface::MockTransport;
using odrive_hardware_interface::axis_endpoint;
using odrive_hardware_interface::make_fake_diagnostics_factory;

hardware_interface::HardwareInfo make_basic_hardware_info()
{
  hardware_interface::HardwareInfo info;
  info.name = "odrive";

  hardware_interface::ComponentInfo sensor;
  sensor.name = "bus";
  sensor.parameters["serial_number"] = "1";
  info.sensors.push_back(sensor);

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = "1";
  joint.parameters["axis"] = "0";
  joint.parameters["watchdog_timeout"] = "0.1";
  joint.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint);

  return info;
}

void configure_default_transport_factory(
  DiagnosticsConfigTestHelper & helper,
  MockTransport * & transport)
{
  helper.set_diagnostics_factory(make_fake_diagnostics_factory());
  helper.set_transport_factory(
    [&]() {
      auto instance = std::make_unique<MockTransport>();
      transport = instance.get();
      const std::int64_t serial = 0x1;
      const int axis = 0;
      const float torque_constant = 6.0F;
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
}

TEST(DiagnosticsConfigTest, PublishDiagnosticsFalseDisablesDiagnostics)
{
  DiagnosticsConfigTestHelper helper;
  MockTransport * transport = nullptr;
  configure_default_transport_factory(helper, transport);

  auto info = make_basic_hardware_info();
  info.hardware_parameters["publish_diagnostics"] = "false";

  ASSERT_TRUE(helper.configure(info));
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());

  EXPECT_FALSE(helper.diagnostics_enabled());
  EXPECT_FALSE(helper.has_diagnostics_node());
  EXPECT_FALSE(helper.has_diagnostics_updater());
  EXPECT_DOUBLE_EQ(0.5, helper.diagnostics_period_seconds());
  EXPECT_DOUBLE_EQ(85.0, helper.warn_threshold());
  EXPECT_DOUBLE_EQ(95.0, helper.error_threshold());
}

TEST(DiagnosticsConfigTest, MissingFactoryDisablesDiagnostics)
{
  DiagnosticsConfigTestHelper helper;
  MockTransport * transport = nullptr;

  helper.set_transport_factory(
    [&]() {
      auto instance = std::make_unique<MockTransport>();
      transport = instance.get();
      const std::int64_t serial = 0x1;
      const int axis = 0;
      const float torque_constant = 6.0F;
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

  auto info = make_basic_hardware_info();
  info.hardware_parameters["publish_diagnostics"] = "true";

  ASSERT_TRUE(helper.configure(info));
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());

  EXPECT_FALSE(helper.diagnostics_enabled());
  EXPECT_FALSE(helper.has_diagnostics_node());
  EXPECT_FALSE(helper.has_diagnostics_updater());
}

TEST(DiagnosticsConfigTest, ValidParametersOverrideDefaults)
{
  DiagnosticsConfigTestHelper helper;
  MockTransport * transport = nullptr;
  configure_default_transport_factory(helper, transport);

  auto info = make_basic_hardware_info();
  info.hardware_parameters["diagnostics_period"] = "1.25";
  info.hardware_parameters["diagnostics_warn_temperature_deg_c"] = "60.5";
  info.hardware_parameters["diagnostics_error_temperature_deg_c"] = "70.5";

  ASSERT_TRUE(helper.configure(info));
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());

  EXPECT_TRUE(helper.diagnostics_enabled());
  EXPECT_TRUE(helper.has_diagnostics_node());
  EXPECT_TRUE(helper.has_diagnostics_updater());
  EXPECT_NEAR(1.25, helper.diagnostics_period_seconds(), 1e-9);
  EXPECT_DOUBLE_EQ(60.5, helper.warn_threshold());
  EXPECT_DOUBLE_EQ(70.5, helper.error_threshold());
}

TEST(DiagnosticsConfigTest, InvalidParametersFallBackToDefaults)
{
  DiagnosticsConfigTestHelper helper;
  MockTransport * transport = nullptr;
  configure_default_transport_factory(helper, transport);

  auto info = make_basic_hardware_info();
  info.hardware_parameters["diagnostics_period"] = "-1.0";
  info.hardware_parameters["diagnostics_warn_temperature_deg_c"] = "invalid";
  info.hardware_parameters["diagnostics_error_temperature_deg_c"] = "oops";

  ASSERT_TRUE(helper.configure(info));
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());

  EXPECT_TRUE(helper.diagnostics_enabled());
  EXPECT_TRUE(helper.has_diagnostics_node());
  EXPECT_TRUE(helper.has_diagnostics_updater());
  EXPECT_DOUBLE_EQ(0.5, helper.diagnostics_period_seconds());
  EXPECT_DOUBLE_EQ(85.0, helper.warn_threshold());
  EXPECT_DOUBLE_EQ(95.0, helper.error_threshold());
}

std::optional<std::string> find_value(
  const diagnostic_updater::DiagnosticStatusWrapper & status,
  const std::string & key)
{
  for (const auto & kv : status.values) {
    if (kv.key == key) {
      return kv.value;
    }
  }
  return std::nullopt;
}

TEST(DiagnosticsTest, JointAggregatesAxisErrorsAndTemperatureFaults)
{
  DiagnosticsTestHelper helper;
  auto & joint = helper.add_joint("steering_joint");
  joint.axis_error = static_cast<double>(0x00000100ULL);
  joint.motor_temperature = helper.error_threshold() + 5.0;

  diagnostic_updater::DiagnosticStatusWrapper status;
  helper.populate_joint_status(status, 0);

  EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::ERROR, status.level);
  EXPECT_NE(std::string::npos, status.message.find("axis: encoder_failed"));
  EXPECT_NE(std::string::npos, status.message.find("temperature.motor_c high"));

  const auto axis_bits = find_value(status, "axis.error_bits");
  ASSERT_TRUE(axis_bits.has_value());
  EXPECT_EQ("0x0000000000000100", axis_bits.value());

  const auto axis_flags = find_value(status, "axis.error_flags");
  ASSERT_TRUE(axis_flags.has_value());
  EXPECT_NE(std::string::npos, axis_flags.value().find("encoder_failed"));
}

TEST(DiagnosticsTest, JointWarnsWhenTemperaturesAreElevated)
{
  DiagnosticsTestHelper helper;
  auto & joint = helper.add_joint("drive_joint");
  joint.fet_temperature = helper.warn_threshold() + 1.5;
  joint.motor_temperature = std::numeric_limits<double>::quiet_NaN();

  diagnostic_updater::DiagnosticStatusWrapper status;
  helper.populate_joint_status(status, 0);

  EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::WARN, status.level);
  EXPECT_NE(std::string::npos, status.message.find("temperature.fet_c elevated"));

  const auto fet_value = find_value(status, "temperature.fet_c");
  ASSERT_TRUE(fet_value.has_value());
  EXPECT_NE(std::string::npos, fet_value.value().find("71."));

  const auto axis_bits = find_value(status, "axis.error_bits");
  ASSERT_TRUE(axis_bits.has_value());
  EXPECT_EQ("0x0", axis_bits.value());
}

TEST(DiagnosticsTest, SensorWarnsWhenVoltageUnavailable)
{
  DiagnosticsTestHelper helper;
  auto & sensor = helper.add_sensor("bus_voltage");
  sensor.vbus_voltage = std::numeric_limits<double>::quiet_NaN();

  diagnostic_updater::DiagnosticStatusWrapper status;
  helper.populate_sensor_status(status, 0);

  EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::WARN, status.level);
  EXPECT_EQ("Voltage unavailable", status.message);

  const auto voltage_value = find_value(status, "vbus_voltage_v");
  ASSERT_TRUE(voltage_value.has_value());
  EXPECT_EQ("NaN", voltage_value.value());
}

TEST(DiagnosticsTest, JointReportsNominalWhenNoFaults)
{
  DiagnosticsTestHelper helper;
  auto & joint = helper.add_joint("idle_joint");
  joint.fet_temperature = 40.0;
  joint.motor_temperature = 35.0;
  joint.axis_error = 0.0;
  joint.motor_error = 0.0;
  joint.encoder_error = 0.0;
  joint.controller_error = 0.0;

  diagnostic_updater::DiagnosticStatusWrapper status;
  helper.populate_joint_status(status, 0);

  EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::OK, status.level);
  EXPECT_EQ("Nominal", status.message);

  const auto fet = find_value(status, "temperature.fet_c");
  ASSERT_TRUE(fet.has_value());
  EXPECT_EQ("40", fet.value());

  const auto motor = find_value(status, "temperature.motor_c");
  ASSERT_TRUE(motor.has_value());
  EXPECT_EQ("35", motor.value());

  const auto axis_bits = find_value(status, "axis.error_bits");
  ASSERT_TRUE(axis_bits.has_value());
  EXPECT_EQ("0x0", axis_bits.value());
}
}  // namespace
