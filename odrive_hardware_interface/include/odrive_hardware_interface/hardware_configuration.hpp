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
#include <vector>

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
  struct CommandLimits
  {
    std::optional<double> position_min;
    std::optional<double> position_max;
    std::optional<double> velocity_min;
    std::optional<double> velocity_max;
    std::optional<double> effort_min;
    std::optional<double> effort_max;
  } command_limits;
};

struct HardwareConfiguration
{
  std::vector<SensorConfig> sensors;
  std::vector<JointConfig> joints;
};
}  // namespace odrive_hardware_interface
