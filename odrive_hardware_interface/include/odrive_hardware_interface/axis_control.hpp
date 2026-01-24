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
#include <string>

#include "odrive_hardware_interface/axis_utils.hpp"
#include "odrive_hardware_interface/odrive_transport.hpp"

namespace odrive_hardware_interface
{
constexpr std::int32_t kAxisStateIdle = 1;
constexpr std::int32_t kAxisStateClosedLoopControl = 8;

enum class AxisControlLevel : std::int32_t
{
  UNDEFINED = 0,
  EFFORT = 1,
  VELOCITY = 2,
  POSITION = 3
};

struct AxisCommandState
{
  double & command_position;
  double & command_velocity;
  double & command_effort;
  const double & state_position;
  const double & state_velocity;
  const double & state_effort;
};

/// Configure the controller for a new command mode and prime initial setpoints.
/// Returns 0 on success.
int perform_axis_mode_switch(
  ODriveTransport & transport,
  std::int64_t serial_number,
  int axis,
  AxisControlLevel level,
  AxisCommandState command_state,
  std::string & failing_stage);

/// Write the latest command setpoints for the current mode.
/// When enable_watchdog is true, feeds the watchdog after sending commands.
///
/// IMPORTANT: Watchdog feeds happen ONLY during successful command writes.
/// This ensures the watchdog detects control loop failures, not just
/// communication issues. If your controller stops commanding (e.g., lifecycle
/// state change, fault masking), the watchdog will expire as intended.
///
/// Returns 0 on success.
int write_axis_command(
  ODriveTransport & transport,
  std::int64_t serial_number,
  int axis,
  AxisControlLevel level,
  const AxisCommandState & command_state,
  bool enable_watchdog,
  std::string & failing_stage);
}  // namespace odrive_hardware_interface
