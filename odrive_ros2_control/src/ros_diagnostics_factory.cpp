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

#include <memory>
#include <string>
#include <utility>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

namespace odrive_ros2_control
{
namespace
{
using DiagnosticsInterface = odrive_hardware_interface::DiagnosticsInterface;

class RosDiagnostics final : public DiagnosticsInterface
{
public:
  using TaskCallback = DiagnosticsInterface::TaskCallback;
  using TimePoint = DiagnosticsInterface::TimePoint;

  explicit RosDiagnostics(const odrive_hardware_interface::DiagnosticsCreationOptions & options)
  {
    rclcpp::NodeOptions node_options;
    node_options.use_intra_process_comms(false);
    node_options.start_parameter_services(false);
    node_options.start_parameter_event_publisher(false);
    node_ = std::make_shared<rclcpp::Node>(options.node_name, node_options);
    updater_ = std::make_shared<diagnostic_updater::Updater>(node_);
    updater_->setHardwareID(options.hardware_id);
  }

  void set_hardware_id(const std::string & hardware_id) override
  {
    updater_->setHardwareID(hardware_id);
  }

  void add_task(const std::string & name, TaskCallback task) override
  {
    updater_->add(name, std::move(task));
  }

  void force_update() override
  {
    updater_->force_update();
  }

  TimePoint now() const override
  {
    return TimePoint{node_->now().nanoseconds()};
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<diagnostic_updater::Updater> updater_;
};
}  // namespace

odrive_hardware_interface::DiagnosticsFactory make_ros_diagnostics_factory()
{
  return [](const odrive_hardware_interface::DiagnosticsCreationOptions & options) {
      return std::make_shared<RosDiagnostics>(options);
    };
}
}  // namespace odrive_ros2_control
