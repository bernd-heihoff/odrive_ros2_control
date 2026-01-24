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

#include "odrive_hardware_interface/command_validation.hpp"

#include <cmath>
#include <optional>
#include <sstream>
#include <utility>

namespace odrive_hardware_interface
{
namespace
{
bool validate_component(
  const char * label,
  double value,
  const std::optional<double> & min_limit,
  const std::optional<double> & max_limit,
  std::ostringstream & message)
{
  if (!std::isfinite(value)) {
    message << label << " command is not finite";
    return false;
  }

  if (min_limit && value < *min_limit) {
    message << label << " command " << value << " is below minimum " << *min_limit;
    return false;
  }
  if (max_limit && value > *max_limit) {
    message << label << " command " << value << " exceeds maximum " << *max_limit;
    return false;
  }

  return true;
}
}  // namespace

bool apply_rate_limiting(
  const char * label,
  double & command,
  double last_command,
  const std::optional<double> & max_rate,
  double dt,
  std::ostringstream & message)
{
  if (!max_rate || !std::isfinite(last_command) || dt <= 0.0) {
    return true;  // No rate limiting
  }

  double delta = command - last_command;
  double max_delta = (*max_rate) * dt;

  if (std::abs(delta) > max_delta) {
    double limited_value = last_command + std::copysign(max_delta, delta);
    message << label << " rate limited from " << command
            << " to " << limited_value
            << " (max_rate=" << *max_rate << " dt=" << dt << ")";
    command = limited_value;
    return false;  // Was rate limited
  }

  return true;
}

bool validate_joint_command(
  const JointConfig::CommandLimits & limits,
  AxisControlLevel level,
  const AxisCommandState & command_state,
  std::string & reason)
{
  std::ostringstream message;

  auto validate_position = [&]() {
      return validate_component(
        "Position",
        command_state.command_position,
        limits.position_min,
        limits.position_max,
        message);
    };

  auto validate_velocity = [&]() {
      return validate_component(
        "Velocity",
        command_state.command_velocity,
        limits.velocity_min,
        limits.velocity_max,
        message);
    };

  auto validate_effort = [&]() {
      return validate_component(
        "Effort",
        command_state.command_effort,
        limits.effort_min,
        limits.effort_max,
        message);
    };

  bool valid = true;
  switch (level) {
    case AxisControlLevel::POSITION:
      valid = validate_position() && validate_velocity() && validate_effort();
      break;
    case AxisControlLevel::VELOCITY:
      valid = validate_velocity() && validate_effort();
      break;
    case AxisControlLevel::EFFORT:
      valid = validate_effort();
      break;
    case AxisControlLevel::UNDEFINED:
      valid = true;
      break;
  }

  if (!valid) {
    reason = message.str();
  } else {
    reason.clear();
  }

  return valid;
}
}  // namespace odrive_hardware_interface
