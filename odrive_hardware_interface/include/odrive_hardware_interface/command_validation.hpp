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

#include <string>

#include "odrive_hardware_interface/axis_control.hpp"
#include "odrive_hardware_interface/odrive_configuration.hpp"

namespace odrive_hardware_interface
{
/// Validate the current joint command against configured limits and active control level.
///\returns true when the command is acceptable. On failure, \p reason contains a description.
bool validate_joint_command(
  const JointConfig::CommandLimits & limits,
  AxisControlLevel level,
  const AxisCommandState & command_state,
  std::string & reason);
}  // namespace odrive_hardware_interface
