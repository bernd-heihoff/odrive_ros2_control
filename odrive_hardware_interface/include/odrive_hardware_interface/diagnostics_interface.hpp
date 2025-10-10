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

#include <functional>
#include <memory>
#include <string>

#include "odrive_hardware_interface/visibility_control.hpp"
#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"

namespace diagnostic_updater
{
class DiagnosticStatusWrapper;
}  // namespace diagnostic_updater

namespace odrive_hardware_interface
{
class DiagnosticsInterface
{
public:
  using TaskCallback = std::function<void(diagnostic_updater::DiagnosticStatusWrapper &)>;

  virtual ~DiagnosticsInterface() = default;

  virtual void set_hardware_id(const std::string & hardware_id) = 0;
  virtual void add_task(const std::string & name, TaskCallback task) = 0;
  virtual void force_update() = 0;
  virtual rclcpp::Time now() const = 0;
  virtual rcl_clock_type_t clock_type() const = 0;
};

struct DiagnosticsCreationOptions
{
  std::string node_name;
  std::string hardware_id;
};

using DiagnosticsFactory = std::function<std::shared_ptr<DiagnosticsInterface>(
  const DiagnosticsCreationOptions & options)>;
}  // namespace odrive_hardware_interface
