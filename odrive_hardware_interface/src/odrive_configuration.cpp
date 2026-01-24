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

#include "odrive_hardware_interface/odrive_configuration.hpp"

#include <algorithm>
#include <cctype>
#include <optional>
#include <unordered_map>

namespace odrive_hardware_interface
{
namespace
{
bool parse_hex_serial(const std::string & value, std::int64_t & serial)
{
  try {
    std::size_t processed = 0;
    const auto parsed = std::stoull(value, &processed, 16);
    if (processed != value.size()) {
      return false;
    }
    serial = static_cast<std::int64_t>(parsed);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool parse_bool(const std::string & value, bool & result)
{
  std::string lowered(value.size(), '\0');
  std::transform(
    value.begin(), value.end(), lowered.begin(), [](unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });

  if (lowered == "true" || lowered == "yes" || lowered == "on") {
    result = true;
    return true;
  }
  if (lowered == "false" || lowered == "no" || lowered == "off") {
    result = false;
    return true;
  }

  try {
    const auto numeric = std::stoi(value);
    result = (numeric != 0);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool parse_int(const std::string & value, int & result)
{
  try {
    std::size_t processed = 0;
    const auto parsed = std::stoi(value, &processed, 10);
    if (processed != value.size()) {
      return false;
    }
    result = parsed;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool parse_double(const std::string & value, double & result)
{
  try {
    std::size_t processed = 0;
    const auto parsed = std::stod(value, &processed);
    if (processed != value.size()) {
      return false;
    }
    result = parsed;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool parse_optional_double(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & key,
  std::optional<double> & target,
  std::string & error_message,
  const std::string & component_name)
{
  const auto it = parameters.find(key);
  if (it == parameters.end()) {
    target.reset();
    return true;
  }

  double parsed_value = 0.0;
  if (!parse_double(it->second, parsed_value)) {
    error_message = "Joint '" + component_name + "' has invalid '" + key + "' value";
    return false;
  }

  target = parsed_value;
  return true;
}
}  // namespace

bool parse_hardware_configuration(
  const hardware_interface::HardwareInfo & info,
  HardwareConfiguration & config,
  std::string & error_message)
{
  HardwareConfiguration parsed;
  parsed.sensors.reserve(info.sensors.size());
  parsed.joints.reserve(info.joints.size());

  // Sensors are optional: deployments that do not monitor bus voltage can omit
  // them and rely on joint telemetry/error state instead.
  if (info.joints.empty()) {
    error_message = "No joints defined in hardware info.";
    return false;
  }

  for (const auto & sensor : info.sensors) {
    const auto serial_it = sensor.parameters.find("serial_number");
    if (serial_it == sensor.parameters.end()) {
      error_message = "Sensor '" + sensor.name + "' missing 'serial_number' parameter";
      return false;
    }

    SensorConfig sensor_config;
    sensor_config.name = sensor.name;
    if (!parse_hex_serial(serial_it->second, sensor_config.serial_number)) {
      error_message = "Sensor '" + sensor.name + "' has invalid 'serial_number' value";
      return false;
    }

    parsed.sensors.emplace_back(sensor_config);
  }

  for (const auto & joint : info.joints) {
    const auto serial_it = joint.parameters.find("serial_number");
    if (serial_it == joint.parameters.end()) {
      error_message = "Joint '" + joint.name + "' missing 'serial_number' parameter";
      return false;
    }

    const auto axis_it = joint.parameters.find("axis");
    if (axis_it == joint.parameters.end()) {
      error_message = "Joint '" + joint.name + "' missing 'axis' parameter";
      return false;
    }

    const auto watchdog_timeout_it = joint.parameters.find("watchdog_timeout");
    if (watchdog_timeout_it == joint.parameters.end()) {
      error_message = "Joint '" + joint.name + "' missing 'watchdog_timeout' parameter";
      return false;
    }

    const auto enable_watchdog_it = joint.parameters.find("enable_watchdog");
    if (enable_watchdog_it == joint.parameters.end()) {
      error_message = "Joint '" + joint.name + "' missing 'enable_watchdog' parameter";
      return false;
    }

    JointConfig joint_config;
    joint_config.name = joint.name;
    if (!parse_hex_serial(serial_it->second, joint_config.serial_number)) {
      error_message = "Joint '" + joint.name + "' has invalid 'serial_number' value";
      return false;
    }
    if (!parse_int(axis_it->second, joint_config.axis)) {
      error_message = "Joint '" + joint.name + "' has invalid 'axis' value";
      return false;
    }
    if (!parse_double(watchdog_timeout_it->second, joint_config.watchdog_timeout)) {
      error_message = "Joint '" + joint.name + "' has invalid 'watchdog_timeout' value";
      return false;
    }
    if (!parse_bool(enable_watchdog_it->second, joint_config.enable_watchdog)) {
      error_message = "Joint '" + joint.name + "' has invalid 'enable_watchdog' value";
      return false;
    }

    // Validate watchdog timeout if enabled
    if (joint_config.enable_watchdog) {
      if (joint_config.watchdog_timeout <= 0.0) {
        error_message = "Joint '" + joint.name + "' watchdog_timeout must be positive";
        return false;
      }
      if (joint_config.watchdog_timeout < 0.01) {
        error_message = "Joint '" + joint.name + "' watchdog_timeout too short (< 10ms)";
        return false;
      }
    }

    joint_config.command_limits.enforce = true;
    const auto enforce_limits_it = joint.parameters.find("enforce_command_limits");
    if (enforce_limits_it != joint.parameters.end()) {
      if (!parse_bool(enforce_limits_it->second, joint_config.command_limits.enforce)) {
        error_message = "Joint '" + joint.name + "' has invalid 'enforce_command_limits' value";
        return false;
      }
    }

    if (!parse_optional_double(
        joint.parameters, "command_position_min",
        joint_config.command_limits.position_min, error_message, joint.name))
    {
      return false;
    }
    if (!parse_optional_double(
        joint.parameters, "command_position_max",
        joint_config.command_limits.position_max, error_message, joint.name))
    {
      return false;
    }
    if (!parse_optional_double(
        joint.parameters, "command_velocity_min",
        joint_config.command_limits.velocity_min, error_message, joint.name))
    {
      return false;
    }
    if (!parse_optional_double(
        joint.parameters, "command_velocity_max",
        joint_config.command_limits.velocity_max, error_message, joint.name))
    {
      return false;
    }
    if (!parse_optional_double(
        joint.parameters, "command_effort_min",
        joint_config.command_limits.effort_min, error_message, joint.name))
    {
      return false;
    }
    if (!parse_optional_double(
        joint.parameters, "command_effort_max",
        joint_config.command_limits.effort_max, error_message, joint.name))
    {
      return false;
    }

    if (joint_config.command_limits.position_min && joint_config.command_limits.position_max &&
      *joint_config.command_limits.position_min > *joint_config.command_limits.position_max)
    {
      error_message = "Joint '" + joint.name + "' has command_position_min greater than "
        "command_position_max";
      return false;
    }
    if (joint_config.command_limits.velocity_min && joint_config.command_limits.velocity_max &&
      *joint_config.command_limits.velocity_min > *joint_config.command_limits.velocity_max)
    {
      error_message = "Joint '" + joint.name + "' has command_velocity_min greater than "
        "command_velocity_max";
      return false;
    }
    if (joint_config.command_limits.effort_min && joint_config.command_limits.effort_max &&
      *joint_config.command_limits.effort_min > *joint_config.command_limits.effort_max)
    {
      error_message = "Joint '" + joint.name + "' has command_effort_min greater than "
        "command_effort_max";
      return false;
    }

    parsed.joints.emplace_back(joint_config);
  }

  config = std::move(parsed);
  error_message.clear();
  return true;
}
}  // namespace odrive_hardware_interface
