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
#include <optional>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "odrive_hardware_interface/odrive_hardware_interface.hpp"

namespace odrive_hardware_interface
{
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

  void populate_joint_status(diagnostic_updater::DiagnosticStatusWrapper & status, std::size_t index) const
  {
    populate_joint_diagnostics(status, index);
  }

  void populate_drive_status(diagnostic_updater::DiagnosticStatusWrapper & status, std::size_t index) const
  {
    populate_drive_diagnostics(status, index);
  }

  void populate_sensor_status(diagnostic_updater::DiagnosticStatusWrapper & status, std::size_t index) const
  {
    populate_sensor_diagnostics(status, index);
  }
};

namespace
{
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
  EXPECT_NE(std::string::npos, status.message.find("Axis: encoder_failed"));
  EXPECT_NE(std::string::npos, status.message.find("Motor temperature [C] high"));

  const auto axis_bits = find_value(status, "Axis error bits");
  ASSERT_TRUE(axis_bits.has_value());
  EXPECT_EQ("0x0000000000000100", axis_bits.value());

  const auto axis_flags = find_value(status, "Axis error flags");
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
  EXPECT_NE(std::string::npos, status.message.find("FET temperature [C] elevated"));

  const auto fet_value = find_value(status, "FET temperature [C]");
  ASSERT_TRUE(fet_value.has_value());
  EXPECT_NE(std::string::npos, fet_value.value().find("71."));

  const auto axis_bits = find_value(status, "Axis error bits");
  ASSERT_TRUE(axis_bits.has_value());
  EXPECT_EQ("0x0", axis_bits.value());
}

TEST(DiagnosticsTest, DriveReportsDecodedOdriveError)
{
  DiagnosticsTestHelper helper;
  auto & drive = helper.add_drive("drive_label");
  drive.odrive_error = static_cast<double>(0x00000010ULL);

  diagnostic_updater::DiagnosticStatusWrapper status;
  helper.populate_drive_status(status, 0);

  EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::ERROR, status.level);
  EXPECT_NE(std::string::npos, status.message.find("dc_bus_over_current"));

  const auto bits = find_value(status, "ODrive error bits");
  ASSERT_TRUE(bits.has_value());
  EXPECT_EQ("0x0000000000000010", bits.value());
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

  const auto voltage_value = find_value(status, "Vbus voltage [V]");
  ASSERT_TRUE(voltage_value.has_value());
  EXPECT_EQ("NaN", voltage_value.value());
}
}  // namespace
}  // namespace odrive_hardware_interface
