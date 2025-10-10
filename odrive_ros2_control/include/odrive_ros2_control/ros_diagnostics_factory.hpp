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

#include "odrive_hardware_interface/diagnostics_interface.hpp"
#include "odrive_ros2_control/visibility_control.hpp"

namespace odrive_ros2_control
{
ODRIVE_ROS2_CONTROL_PUBLIC odrive_hardware_interface::DiagnosticsFactory make_ros_diagnostics_factory();
}  // namespace odrive_ros2_control

