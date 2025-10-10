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

#include <memory>
#include <string>

#include "odrive_hardware_interface/diagnostics_interface.hpp"

namespace odrive_hardware_interface
{
class FakeDiagnostics final : public DiagnosticsInterface
{
public:
  void set_hardware_id(const std::string &) override {}

  void add_task(const std::string &, TaskCallback) override {}

  void force_update() override {}

  rclcpp::Time now() const override
  {
    return rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  rcl_clock_type_t clock_type() const override
  {
    return RCL_ROS_TIME;
  }
};

inline DiagnosticsFactory make_fake_diagnostics_factory()
{
  return [](const DiagnosticsCreationOptions &) {
      return std::make_shared<FakeDiagnostics>();
    };
}
}  // namespace odrive_hardware_interface
