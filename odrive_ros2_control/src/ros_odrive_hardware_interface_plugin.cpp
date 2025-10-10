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

#include "odrive_ros2_control/ros_diagnostics_factory.hpp"

#include "odrive_hardware_interface/odrive_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace odrive_ros2_control
{
class RosODriveHardwareInterface : public odrive_hardware_interface::ODriveHardwareInterface
{
public:
  RosODriveHardwareInterface()
  {
    set_diagnostics_factory(make_ros_diagnostics_factory());
  }
};
}  // namespace odrive_ros2_control

PLUGINLIB_EXPORT_CLASS(
  odrive_ros2_control::RosODriveHardwareInterface, hardware_interface::SystemInterface)
