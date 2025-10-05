// Copyright 2021 Factor Robotics
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

#include "odrive_hardware_interface/odrive_hardware_interface.hpp"

#include <utility>

#include "odrive_hardware_interface/axis_control.hpp"
#include "odrive_hardware_interface/axis_telemetry.hpp"
#include "odrive_hardware_interface/axis_utils.hpp"
#include "odrive_hardware_interface/odrive_usb.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace odrive_hardware_interface
{
namespace
{
constexpr const char kLoggerName[] = "ODriveHardwareInterface";

CallbackReturn to_callback_return(int status, const std::string & action)
{
  if (status == 0) {
    return CallbackReturn::SUCCESS;
  }

  RCLCPP_ERROR(
    rclcpp::get_logger(kLoggerName), "Transport error (%d) while %s", status, action.c_str());
  return CallbackReturn::ERROR;
}

return_type to_io_return(int status, const std::string & action)
{
  if (status == 0) {
    return return_type::OK;
  }

  RCLCPP_ERROR(
    rclcpp::get_logger(kLoggerName), "Transport error (%d) while %s", status, action.c_str());
  return return_type::ERROR;
}

}  // namespace

CallbackReturn ODriveHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  std::string parse_error;
  HardwareConfiguration parsed_config;
  if (!parse_hardware_configuration(info_, parsed_config, parse_error)) {
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", parse_error.c_str());
    return CallbackReturn::ERROR;
  }
  hardware_config_ = std::move(parsed_config);

  serial_numbers_.clear();
  serial_numbers_.resize(2);
  serial_numbers_[0].reserve(hardware_config_.sensors.size());
  serial_numbers_[1].reserve(hardware_config_.joints.size());

  axes_.clear();
  axes_.reserve(hardware_config_.joints.size());
  torque_constants_.clear();
  torque_constants_.reserve(hardware_config_.joints.size());
  enable_watchdogs_.clear();
  enable_watchdogs_.reserve(hardware_config_.joints.size());

  hw_vbus_voltages_.resize(info_.sensors.size(), std::numeric_limits<double>::quiet_NaN());

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  hw_axis_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_motor_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_encoder_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_controller_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_fet_temperatures_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_motor_temperatures_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const auto & sensor : hardware_config_.sensors) {
    serial_numbers_[0].emplace_back(sensor.serial_number);
  }

  for (const auto & joint : hardware_config_.joints) {
    serial_numbers_[1].emplace_back(joint.serial_number);
    axes_.emplace_back(joint.axis);
    enable_watchdogs_.emplace_back(joint.enable_watchdog);
  }

  transport_ = std::make_unique<odrive::ODriveUSB>();
  if (!transport_) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName), "Failed to create ODriveUSB instance");
    return CallbackReturn::ERROR;
  }
  const auto init_status = transport_->initialize(serial_numbers_);
  if (init_status != 0) {
    return to_callback_return(init_status, "initialising ODrive transport");
  }

  for (size_t i = 0; i < info_.joints.size(); i++) {
    float torque_constant;
    const int read_status = transport_->read(
      serial_numbers_[1][i],
      axis_endpoint(odrive::AXIS__MOTOR__CONFIG__TORQUE_CONSTANT, axes_[i]), torque_constant);
    if (read_status != 0) {
      return to_callback_return(read_status, "reading motor torque constant");
    }
    torque_constants_.emplace_back(torque_constant);

    if (enable_watchdogs_[i]) {
      const int write_timeout_status = transport_->write(
        serial_numbers_[1][i],
        axis_endpoint(odrive::AXIS__CONFIG__WATCHDOG_TIMEOUT, axes_[i]),
        static_cast<float>(hardware_config_.joints[i].watchdog_timeout));
      if (write_timeout_status != 0) {
        return to_callback_return(write_timeout_status, "configuring watchdog timeout");
      }
    }
    const int write_enable_status = transport_->write(
      serial_numbers_[1][i],
      axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, axes_[i]),
      static_cast<bool>(enable_watchdogs_[i]));
    if (write_enable_status != 0) {
      return to_callback_return(write_enable_status, "enabling watchdog");
    }
  }

  control_level_.resize(info_.joints.size(), AxisControlLevel::UNDEFINED);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  for (size_t i = 0; i < info_.joints.size(); i++) {
    const std::int64_t serial = serial_numbers_[1][i];
    if (enable_watchdogs_[i]) {
      const int feed_status = transport_->call(
        serial, axis_endpoint(odrive::AXIS__WATCHDOG_FEED, axes_[i]));
      if (feed_status != 0) {
        return to_callback_return(feed_status, "feeding watchdog on activation");
      }
    }
    const int clear_status = transport_->call(
      serial, axis_endpoint(odrive::AXIS__CLEAR_ERRORS, axes_[i]));
    if (clear_status != 0) {
      return to_callback_return(clear_status, "clearing axis errors on activation");
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  int32_t requested_state = kAxisStateIdle;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    const int status = transport_->write(
      serial_numbers_[1][i], axis_endpoint(odrive::AXIS__REQUESTED_STATE, axes_[i]),
      requested_state);
    if (status != 0) {
      return to_callback_return(status, "requesting axis idle state");
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.sensors.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.sensors[i].name, "vbus_voltage", &hw_vbus_voltages_[i]));
  }

  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[i].name, "axis_error", &hw_axis_errors_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "motor_error", &hw_motor_errors_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "encoder_error", &hw_encoder_errors_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "controller_error", &hw_controller_errors_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "fet_temperature", &hw_fet_temperatures_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "motor_temperature", &hw_motor_temperatures_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ODriveHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
  }

  return command_interfaces;
}

return_type ODriveHardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (std::string key : stop_interfaces) {
    for (size_t i = 0; i < info_.joints.size(); i++) {
      if (key.find(info_.joints[i].name) != std::string::npos) {
        control_level_[i] = AxisControlLevel::UNDEFINED;
      }
    }
  }

  for (std::string key : start_interfaces) {
    for (size_t i = 0; i < info_.joints.size(); i++) {
      switch (control_level_[i]) {
        case AxisControlLevel::UNDEFINED:
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
            control_level_[i] = AxisControlLevel::EFFORT;
          }
          break;
        case AxisControlLevel::EFFORT:
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
            control_level_[i] = AxisControlLevel::VELOCITY;
          }
          break;
        case AxisControlLevel::VELOCITY:
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
            control_level_[i] = AxisControlLevel::POSITION;
          }
          break;
        case AxisControlLevel::POSITION:
          break;
      }
    }
  }

  return return_type::OK;
}

return_type ODriveHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> &, const std::vector<std::string> &)
{
  for (size_t i = 0; i < info_.joints.size(); i++) {
    AxisCommandState command_state{
      hw_commands_positions_[i],
      hw_commands_velocities_[i],
      hw_commands_efforts_[i],
      hw_positions_[i],
      hw_velocities_[i],
      hw_efforts_[i]};
    std::string failing_stage;
    if (const int status = perform_axis_mode_switch(
        *transport_, serial_numbers_[1][i], axes_[i], control_level_[i], command_state,
        failing_stage);
      status != 0)
    {
      std::string action = failing_stage.empty() ? "performing axis mode switch" :
        "performing axis mode switch (" + failing_stage + ")";
      return to_io_return(status, action);
    }
  }

  return return_type::OK;
}

return_type ODriveHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < info_.sensors.size(); i++) {
    float vbus_voltage;

    if (const int status =
      transport_->read(serial_numbers_[0][i], odrive::VBUS_VOLTAGE, vbus_voltage);
      status != 0)
    {
      return to_io_return(status, "reading vbus voltage");
    }
    hw_vbus_voltages_[i] = vbus_voltage;
  }

  for (size_t i = 0; i < info_.joints.size(); i++) {
    AxisTelemetryBuffers buffers{
      hw_efforts_[i],
      hw_velocities_[i],
      hw_positions_[i],
      hw_axis_errors_[i],
      hw_motor_errors_[i],
      hw_encoder_errors_[i],
      hw_controller_errors_[i],
      hw_fet_temperatures_[i],
      hw_motor_temperatures_[i]};
    std::string failing_stage;
    if (const int status = read_axis_telemetry(
        *transport_, serial_numbers_[1][i], axes_[i], torque_constants_[i], buffers,
        failing_stage);
      status != 0)
    {
      std::string action = failing_stage.empty() ? "reading axis telemetry" :
        "reading axis telemetry (" + failing_stage + ")";
      return to_io_return(status, action);
    }
  }

  return return_type::OK;
}

return_type ODriveHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < info_.joints.size(); i++) {
    AxisCommandState command_state{
      hw_commands_positions_[i],
      hw_commands_velocities_[i],
      hw_commands_efforts_[i],
      hw_positions_[i],
      hw_velocities_[i],
      hw_efforts_[i]};
    std::string failing_stage;
    if (const int status = write_axis_command(
        *transport_, serial_numbers_[1][i], axes_[i], control_level_[i], command_state,
        enable_watchdogs_[i], failing_stage);
      status != 0)
    {
      std::string action = failing_stage.empty() ? "writing axis command" :
        "writing axis command (" + failing_stage + ")";
      return to_io_return(status, action);
    }
  }

  return return_type::OK;
}
}  // namespace odrive_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  odrive_hardware_interface::ODriveHardwareInterface, hardware_interface::SystemInterface)
