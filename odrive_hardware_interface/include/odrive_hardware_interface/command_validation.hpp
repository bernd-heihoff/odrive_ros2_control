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

#pragma once

#include <sstream>
#include <string>

#include "odrive_hardware_interface/axis_control.hpp"
#include "odrive_hardware_interface/hardware_configuration.hpp"

namespace odrive_hardware_interface
{
/// Apply rate limiting to a command value.
///
/// Limits the rate of change of a command to prevent abrupt setpoint jumps
/// that could damage hardware or cause instability.
///
/// @param label Human-readable label for logging
/// @param[in,out] command Command value to limit (modified in place)
/// @param last_command Previous command value for delta calculation
/// @param max_rate Maximum allowed rate of change [units/second], or nullopt to disable
/// @param dt Time delta since last command [seconds]
/// @param[out] message Stream for rate limiting notification message
/// @return true if no limiting applied, false if command was rate-limited
bool apply_rate_limiting(
  const char * label,
  double & command,
  double last_command,
  const std::optional<double> & max_rate,
  double dt,
  std::ostringstream & message);

/// Validate the current joint command against configured limits and active control level.
///
/// Performs comprehensive validation:
/// - Checks for NaN and infinite values
/// - Validates against min/max limits for position, velocity, and effort
/// - Only validates fields relevant to the current control level
///
/// @param limits Configured command limits for this joint
/// @param level Active control level (determines which fields to validate)
/// @param command_state Command values to validate
/// @param[out] reason Human-readable failure reason (empty if valid)
/// @return true if command is acceptable, false otherwise
bool validate_joint_command(
  const JointConfig::CommandLimits & limits,
  AxisControlLevel level,
  const AxisCommandState & command_state,
  std::string & reason);
}  // namespace odrive_hardware_interface
