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
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "odrive_hardware_interface/visibility_control.hpp"

namespace odrive_hardware_interface
{
struct SensorConfig
{
  std::string name;
  std::int64_t serial_number;
};

struct JointConfig
{
  std::string name;
  std::int64_t serial_number;
  int axis;
  bool enable_watchdog;
  double watchdog_timeout;
};

struct HardwareConfiguration
{
  std::vector<SensorConfig> sensors;
  std::vector<JointConfig> joints;
};

/// Parse sensor and joint configuration from a hardware description.
/// \param info Hardware info provided by ROS control.
/// \param[out] config Parsed configuration on success.
/// \param[out] error_message Description when parsing fails.
/// \returns true when parsing succeeds.
ODRIVE_HARDWARE_INTERFACE_PUBLIC bool parse_hardware_configuration(
  const hardware_interface::HardwareInfo & info,
  HardwareConfiguration & config,
  std::string & error_message);
}  // namespace odrive_hardware_interface
