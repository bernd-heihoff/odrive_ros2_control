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

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "odrive_hardware_interface/axis_control.hpp"
#include "odrive_hardware_interface/axis_telemetry.hpp"
#include "odrive_hardware_interface/axis_utils.hpp"
#include "odrive_hardware_interface/command_validation.hpp"
#include "odrive_hardware_interface/error_monitoring.hpp"
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

ODriveHardwareInterface::ODriveHardwareInterface()
: transport_factory_([]() {
      return std::make_unique<odrive::ODriveUSB>();
    })
{
}

void ODriveHardwareInterface::set_transport_factory(TransportFactory factory)
{
  transport_factory_ = std::move(factory);
}

void ODriveHardwareInterface::reset_runtime_state()
{
  for (auto & sensor : sensors_) {
    sensor.vbus_voltage = std::numeric_limits<double>::quiet_NaN();
  }

  for (auto & joint : joints_) {
    joint.command_position = std::numeric_limits<double>::quiet_NaN();
    joint.command_velocity = std::numeric_limits<double>::quiet_NaN();
    joint.command_effort = std::numeric_limits<double>::quiet_NaN();

    joint.position = std::numeric_limits<double>::quiet_NaN();
    joint.velocity = std::numeric_limits<double>::quiet_NaN();
    joint.effort = std::numeric_limits<double>::quiet_NaN();

    joint.axis_error = std::numeric_limits<double>::quiet_NaN();
    joint.motor_error = std::numeric_limits<double>::quiet_NaN();
    joint.encoder_error = std::numeric_limits<double>::quiet_NaN();
    joint.controller_error = std::numeric_limits<double>::quiet_NaN();
    joint.fet_temperature = std::numeric_limits<double>::quiet_NaN();
    joint.motor_temperature = std::numeric_limits<double>::quiet_NaN();

    joint.torque_constant = std::numeric_limits<float>::quiet_NaN();
    joint.control_level = AxisControlLevel::UNDEFINED;
    joint.last_axis_error = 0;
    joint.last_motor_error = 0;
    joint.last_encoder_error = 0;
    joint.last_controller_error = 0;
  }
}

CallbackReturn ODriveHardwareInterface::initialize_transport()
{
  if (hardware_config_.joints.size() != joints_.size() ||
    hardware_config_.sensors.size() != sensors_.size())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Hardware configuration does not match runtime context (joints: %zu/%zu, sensors: %zu/%zu)",
      hardware_config_.joints.size(), joints_.size(),
      hardware_config_.sensors.size(), sensors_.size());
    return CallbackReturn::ERROR;
  }

  transport_ = transport_factory_ ? transport_factory_() : nullptr;
  if (!transport_) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName), "Failed to create ODrive transport instance");
    return CallbackReturn::ERROR;
  }

  ODriveTransport::SerialMatrix serial_numbers(2);
  serial_numbers[0].reserve(sensors_.size());
  for (const auto & sensor : sensors_) {
    serial_numbers[0].emplace_back(sensor.serial_number);
  }
  serial_numbers[1].reserve(joints_.size());
  for (const auto & joint : joints_) {
    serial_numbers[1].emplace_back(joint.serial_number);
  }

  const auto init_status = transport_->initialize(serial_numbers);
  if (init_status != 0) {
    return to_callback_return(init_status, "initialising ODrive transport");
  }

  for (size_t i = 0; i < joints_.size(); i++) {
    auto & joint_context = joints_[i];
    const auto & joint_config = hardware_config_.joints[i];

    float torque_constant = std::numeric_limits<float>::quiet_NaN();
    const int read_status = transport_->read(
      joint_context.serial_number,
      axis_endpoint(odrive::AXIS__MOTOR__CONFIG__TORQUE_CONSTANT, joint_context.axis),
      torque_constant);
    if (read_status != 0) {
      return to_callback_return(read_status, "reading motor torque constant");
    }
    joint_context.torque_constant = torque_constant;

    if (joint_context.enable_watchdog) {
      const int write_timeout_status = transport_->write(
        joint_context.serial_number,
        axis_endpoint(odrive::AXIS__CONFIG__WATCHDOG_TIMEOUT, joint_context.axis),
        static_cast<float>(joint_config.watchdog_timeout));
      if (write_timeout_status != 0) {
        return to_callback_return(write_timeout_status, "configuring watchdog timeout");
      }
    }

    const int write_enable_status = transport_->write(
      joint_context.serial_number,
      axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, joint_context.axis),
      static_cast<bool>(joint_context.enable_watchdog));
    if (write_enable_status != 0) {
      return to_callback_return(write_enable_status, "enabling watchdog");
    }
  }

  return CallbackReturn::SUCCESS;
}

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

  sensors_.clear();
  sensors_.reserve(hardware_config_.sensors.size());
  for (const auto & sensor_config : hardware_config_.sensors) {
    SensorContext sensor;
    sensor.serial_number = sensor_config.serial_number;
    sensors_.emplace_back(sensor);
  }

  joints_.clear();
  joints_.reserve(hardware_config_.joints.size());
  for (const auto & joint_config : hardware_config_.joints) {
    ODriveHardwareInterface::JointContext joint;
    joint.serial_number = joint_config.serial_number;
    joint.axis = joint_config.axis;
    joint.enable_watchdog = joint_config.enable_watchdog;
    joint.command_limits = joint_config.command_limits;
    joints_.emplace_back(joint);
  }

  return initialize_transport();
}

CallbackReturn ODriveHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  for (auto & joint : joints_) {
    if (joint.enable_watchdog) {
      const int feed_status = transport_->call(
        joint.serial_number, axis_endpoint(odrive::AXIS__WATCHDOG_FEED, joint.axis));
      if (feed_status != 0) {
        return to_callback_return(feed_status, "feeding watchdog on activation");
      }
    }
    const int clear_status = transport_->call(
      joint.serial_number, axis_endpoint(odrive::AXIS__CLEAR_ERRORS, joint.axis));
    if (clear_status != 0) {
      return to_callback_return(clear_status, "clearing axis errors on activation");
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  constexpr std::int32_t requested_state = kAxisStateIdle;
  for (const auto & joint : joints_) {
    const int status = transport_->write(
      joint.serial_number, axis_endpoint(odrive::AXIS__REQUESTED_STATE, joint.axis),
      requested_state);
    if (status != 0) {
      return to_callback_return(status, "requesting axis idle state");
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  transport_.reset();
  reset_runtime_state();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::recover()
{
  reset_runtime_state();
  transport_.reset();

  const auto init_result = initialize_transport();
  if (init_result != CallbackReturn::SUCCESS) {
    return init_result;
  }

  for (const auto & joint : joints_) {
    const int clear_status = transport_->call(
      joint.serial_number, axis_endpoint(odrive::AXIS__CLEAR_ERRORS, joint.axis));
    if (clear_status != 0) {
      return to_callback_return(clear_status, "clearing axis errors during recovery");
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
        info_.sensors[i].name, "vbus_voltage", &sensors_[i].vbus_voltage));
  }

  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].effort));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].velocity));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "axis_error", &joints_[i].axis_error));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "motor_error", &joints_[i].motor_error));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "encoder_error", &joints_[i].encoder_error));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "controller_error", &joints_[i].controller_error));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "fet_temperature", &joints_[i].fet_temperature));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "motor_temperature", &joints_[i].motor_temperature));
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
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].command_effort));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command_velocity));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command_position));
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
        joints_[i].control_level = AxisControlLevel::UNDEFINED;
      }
    }
  }

  for (std::string key : start_interfaces) {
    for (size_t i = 0; i < info_.joints.size(); i++) {
      switch (joints_[i].control_level) {
        case AxisControlLevel::UNDEFINED:
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
            joints_[i].control_level = AxisControlLevel::EFFORT;
          }
          break;
        case AxisControlLevel::EFFORT:
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
            joints_[i].control_level = AxisControlLevel::VELOCITY;
          }
          break;
        case AxisControlLevel::VELOCITY:
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
            joints_[i].control_level = AxisControlLevel::POSITION;
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
      joints_[i].command_position,
      joints_[i].command_velocity,
      joints_[i].command_effort,
      joints_[i].position,
      joints_[i].velocity,
      joints_[i].effort};
    std::string failing_stage;
    if (const int status = perform_axis_mode_switch(
        *transport_, joints_[i].serial_number, joints_[i].axis, joints_[i].control_level,
        command_state,
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
      transport_->read(sensors_[i].serial_number, odrive::VBUS_VOLTAGE, vbus_voltage);
      status != 0)
    {
      return to_io_return(status, "reading vbus voltage");
    }
    sensors_[i].vbus_voltage = vbus_voltage;
  }

  for (size_t i = 0; i < info_.joints.size(); i++) {
    AxisTelemetryBuffers buffers{
      joints_[i].effort,
      joints_[i].velocity,
      joints_[i].position,
      joints_[i].axis_error,
      joints_[i].motor_error,
      joints_[i].encoder_error,
      joints_[i].controller_error,
      joints_[i].fet_temperature,
      joints_[i].motor_temperature};
    std::string failing_stage;
    if (const int status = read_axis_telemetry(
        *transport_, joints_[i].serial_number, joints_[i].axis, joints_[i].torque_constant,
        buffers,
        failing_stage);
      status != 0)
    {
      std::string action = failing_stage.empty() ? "reading axis telemetry" :
        "reading axis telemetry (" + failing_stage + ")";
      return to_io_return(status, action);
    }

    if (const auto axis_error_value = extract_error_value(joints_[i].axis_error)) {
      log_axis_error_transition(
        info_.joints[i].name,
        *axis_error_value,
        joints_[i].last_axis_error);
    }
    if (const auto motor_error_value = extract_error_value(joints_[i].motor_error)) {
      log_motor_error_transition(
        info_.joints[i].name,
        *motor_error_value,
        joints_[i].last_motor_error);
    }
    if (const auto encoder_error_value = extract_error_value(joints_[i].encoder_error)) {
      log_encoder_error_transition(
        info_.joints[i].name,
        *encoder_error_value,
        joints_[i].last_encoder_error);
    }
    if (const auto controller_error_value = extract_error_value(joints_[i].controller_error)) {
      log_controller_error_transition(
        info_.joints[i].name,
        *controller_error_value,
        joints_[i].last_controller_error);
    }
  }

  return return_type::OK;
}

return_type ODriveHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < info_.joints.size(); i++) {
    AxisCommandState command_state{
      joints_[i].command_position,
      joints_[i].command_velocity,
      joints_[i].command_effort,
      joints_[i].position,
      joints_[i].velocity,
      joints_[i].effort};

    std::string validation_error;
    if (!validate_joint_command(
        joints_[i].command_limits, joints_[i].control_level, command_state, validation_error))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Rejected command for joint '%s': %s",
        info_.joints[i].name.c_str(),
        validation_error.c_str());
      return return_type::ERROR;
    }

    std::string failing_stage;
    if (const int status = write_axis_command(
        *transport_, joints_[i].serial_number, joints_[i].axis, joints_[i].control_level,
        command_state, joints_[i].enable_watchdog, failing_stage);
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
