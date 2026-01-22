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

#include <cstdint>
#include <optional>
#include <string>

namespace odrive_hardware_interface
{
/// Convert a floating-point error value into an integer bitmask if finite and non-zero.
std::optional<std::uint64_t> extract_error_value(double value);

/// Produce a comma-separated textual description for axis error flags.
std::string describe_axis_error(std::uint64_t value);

/// Produce a comma-separated textual description for motor error flags.
std::string describe_motor_error(std::uint64_t value);

/// Produce a comma-separated textual description for encoder error flags.
std::string describe_encoder_error(std::uint64_t value);

/// Produce a comma-separated textual description for controller error flags.
std::string describe_controller_error(std::uint64_t value);

/// Log axis error bit transitions for a joint using the rcutils logging system.
void log_axis_error_transition(
  const std::string & joint_name,
  std::uint64_t current,
  std::uint64_t & last);

/// Log motor error bit transitions for a joint using the rcutils logging system.
void log_motor_error_transition(
  const std::string & joint_name,
  std::uint64_t current,
  std::uint64_t & last);

/// Log encoder error bit transitions for a joint using the rcutils logging system.
void log_encoder_error_transition(
  const std::string & joint_name,
  std::uint64_t current,
  std::uint64_t & last);

/// Log controller error bit transitions for a joint using the rcutils logging system.
void log_controller_error_transition(
  const std::string & joint_name,
  std::uint64_t current,
  std::uint64_t & last);
}  // namespace odrive_hardware_interface
