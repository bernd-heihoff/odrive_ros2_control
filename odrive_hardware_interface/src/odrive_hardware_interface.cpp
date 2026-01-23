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

#include <inttypes.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "odrive_hardware_interface/axis_control.hpp"
#include "odrive_hardware_interface/axis_telemetry.hpp"
#include "odrive_hardware_interface/axis_utils.hpp"
#include "odrive_hardware_interface/command_validation.hpp"
#include "odrive_hardware_interface/error_monitoring.hpp"
#include "odrive_hardware_interface/odrive_usb.hpp"
#include "rcutils/logging_macros.h"

namespace odrive_hardware_interface
{

namespace
{
constexpr const char kLoggerName[] = "ODriveHardwareInterface";
constexpr const char * kDiagnosticsNodePrefix = "odrive_diagnostics_";

DiagnosticsInterface::Duration seconds_to_duration(double seconds)
{
  return std::chrono::duration_cast<DiagnosticsInterface::Duration>(
    std::chrono::duration<double>(seconds));
}

bool try_parse_bool(const std::string & value, bool & result)
{
  std::string lowered(value.size(), '\0');
  std::transform(
    value.begin(), value.end(), lowered.begin(), [](unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });

  if (lowered == "true" || lowered == "yes" || lowered == "on") {
    result = true;
    return true;
  }
  if (lowered == "false" || lowered == "no" || lowered == "off") {
    result = false;
    return true;
  }

  try {
    result = std::stoi(value) != 0;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool try_parse_double(const std::string & value, double & result)
{
  try {
    std::size_t processed = 0;
    const double parsed = std::stod(value, &processed);
    if (processed != value.size()) {
      return false;
    }
    result = parsed;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

std::string sanitize_node_suffix(const std::string & input)
{
  std::string output;
  output.reserve(input.size());
  for (char ch : input) {
    if (std::isalnum(static_cast<unsigned char>(ch)) || ch == '_' || ch == '-') {
      output.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
    } else {
      output.push_back('_');
    }
  }

  if (output.empty()) {
    output = "odrive";
  }

  return output;
}

CallbackReturn to_callback_return(int status, const std::string & action)
{
  if (status == 0) {
    return CallbackReturn::SUCCESS;
  }

  RCUTILS_LOG_ERROR_NAMED(
    kLoggerName, "Transport error (%d) while %s", status, action.c_str());
  return CallbackReturn::ERROR;
}

return_type to_io_return(int status, const std::string & action)
{
  if (status == 0) {
    return return_type::OK;
  }

  RCUTILS_LOG_ERROR_NAMED(
    kLoggerName, "Transport error (%d) while %s", status, action.c_str());
  return return_type::ERROR;
}

}  // namespace

ODriveHardwareInterface::ODriveHardwareInterface()
: transport_factory_([]() {
      return std::make_unique<odrive::ODriveUSB>();
    })
{
}

ODriveHardwareInterface::~ODriveHardwareInterface()
{
}

void ODriveHardwareInterface::set_transport_factory(TransportFactory factory)
{
  transport_factory_ = std::move(factory);
}

void ODriveHardwareInterface::set_diagnostics_factory(DiagnosticsFactory factory)
{
  diagnostics_factory_ = std::move(factory);
}

void ODriveHardwareInterface::reset_runtime_state()
{
  outputs_enabled_ = false;

  for (auto & sensor : sensors_) {
    sensor.vbus_voltage = std::numeric_limits<double>::quiet_NaN();
    sensor.transport_error = 0.0;
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

    joint.read_error = 0.0;
    joint.write_error = 0.0;
    joint.telemetry_valid = 0.0;
    joint.healthy = 0.0;

    joint.torque_constant = std::numeric_limits<float>::quiet_NaN();
    joint.control_level = AxisControlLevel::UNDEFINED;
    joint.last_axis_error = 0;
    joint.last_motor_error = 0;
    joint.last_encoder_error = 0;
    joint.last_controller_error = 0;
  }

  last_diagnostics_update_.reset();

  axis_faulted_.assign(joints_.size(), false);
  idle_requested_on_fault_.assign(joints_.size(), false);
}

CallbackReturn ODriveHardwareInterface::initialize_transport()
{
  if (hardware_config_.joints.size() != joints_.size() ||
    hardware_config_.sensors.size() != sensors_.size())
  {
    RCUTILS_LOG_ERROR_NAMED(
      kLoggerName,
      "Hardware configuration does not match runtime context (joints: %zu/%zu, sensors: %zu/%zu)",
      hardware_config_.joints.size(), joints_.size(),
      hardware_config_.sensors.size(), sensors_.size());
    return CallbackReturn::ERROR;
  }

  transport_ = transport_factory_ ? transport_factory_() : nullptr;
  if (!transport_) {
    RCUTILS_LOG_ERROR_NAMED(
      kLoggerName, "Failed to create ODrive transport instance");
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

    if (joint_context.enable_watchdog) {
      const int feed_status = transport_->call(
        joint_context.serial_number,
        axis_endpoint(odrive::AXIS__WATCHDOG_FEED, joint_context.axis));
      if (feed_status != 0) {
        return to_callback_return(feed_status, "feeding watchdog during initialization");
      }
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  return configure_from_info(info);
}

CallbackReturn ODriveHardwareInterface::configure_from_info(
  const hardware_interface::HardwareInfo & info)
{
  (void)info;

  std::string parse_error;
  HardwareConfiguration parsed_config;
  if (!parse_hardware_configuration(info_, parsed_config, parse_error)) {
    RCUTILS_LOG_ERROR_NAMED(kLoggerName, "%s", parse_error.c_str());
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

  drives_.clear();
  drives_.reserve(hardware_config_.joints.size());
  for (const auto & joint_config : hardware_config_.joints) {
    const auto serial_number = joint_config.serial_number;
    const auto existing = std::find_if(
      drives_.begin(), drives_.end(),
      [&](const DriveContext & drive) {
        return drive.serial_number == serial_number;
      });
    if (existing != drives_.end()) {
      continue;
    }

    DriveContext drive;
    drive.serial_number = serial_number;
    if (joint_config.name.empty()) {
      drive.label = "drive:" + std::to_string(serial_number);
    } else {
      drive.label = joint_config.name + "/drive";
    }
    drives_.emplace_back(std::move(drive));
  }

  diagnostics_.reset();
  diagnostics_config_ = DiagnosticsConfig{};
  diagnostics_period_ = seconds_to_duration(diagnostics_config_.period_sec);
  last_diagnostics_update_.reset();

  safety_config_ = SafetyConfig{};

  {
    auto params = info_.hardware_parameters;

    auto bool_it = params.find("gate_outputs_with_lifecycle");
    if (bool_it != params.end()) {
      bool parsed = safety_config_.gate_outputs_with_lifecycle;
      if (!try_parse_bool(bool_it->second, parsed)) {
        RCUTILS_LOG_ERROR_NAMED(
          kLoggerName, "Invalid 'gate_outputs_with_lifecycle' value '%s'", bool_it->second.c_str());
        return CallbackReturn::ERROR;
      }
      safety_config_.gate_outputs_with_lifecycle = parsed;
    }

    bool_it = params.find("mask_faulted_axes");
    if (bool_it != params.end()) {
      bool parsed = safety_config_.mask_faulted_axes;
      if (!try_parse_bool(bool_it->second, parsed)) {
        RCUTILS_LOG_ERROR_NAMED(
          kLoggerName, "Invalid 'mask_faulted_axes' value '%s'", bool_it->second.c_str());
        return CallbackReturn::ERROR;
      }
      safety_config_.mask_faulted_axes = parsed;
    }

    bool_it = params.find("request_idle_on_axis_fault");
    if (bool_it != params.end()) {
      bool parsed = safety_config_.request_idle_on_axis_fault;
      if (!try_parse_bool(bool_it->second, parsed)) {
        RCUTILS_LOG_ERROR_NAMED(
          kLoggerName, "Invalid 'request_idle_on_axis_fault' value '%s'", bool_it->second.c_str());
        return CallbackReturn::ERROR;
      }
      safety_config_.request_idle_on_axis_fault = parsed;
    }
  }

  {
    auto params = info_.hardware_parameters;

    auto bool_it = params.find("publish_diagnostics");
    if (bool_it != params.end()) {
      bool enabled = diagnostics_config_.enabled;
      if (try_parse_bool(bool_it->second, enabled)) {
        diagnostics_config_.enabled = enabled;
      } else {
        RCUTILS_LOG_WARN_NAMED(
          kLoggerName,
          "Invalid publish_diagnostics value '%s'; keeping default", bool_it->second.c_str());
      }
    }

    auto period_it = params.find("diagnostics_period");
    if (period_it != params.end()) {
      double parsed_period = diagnostics_config_.period_sec;
      if (try_parse_double(period_it->second, parsed_period) && parsed_period > 0.0) {
        diagnostics_config_.period_sec = parsed_period;
      } else {
        RCUTILS_LOG_WARN_NAMED(
          kLoggerName,
          "Invalid diagnostics_period value '%s'; keeping default", period_it->second.c_str());
      }
    }

    auto warn_it = params.find("diagnostics_warn_temperature_deg_c");
    if (warn_it != params.end()) {
      double parsed = diagnostics_config_.warn_temperature_deg_c;
      if (try_parse_double(warn_it->second, parsed)) {
        diagnostics_config_.warn_temperature_deg_c = parsed;
      } else {
        RCUTILS_LOG_WARN_NAMED(
          kLoggerName,
          "Invalid diagnostics_warn_temperature_deg_c '%s'; keeping default",
          warn_it->second.c_str());
      }
    }

    auto error_it = params.find("diagnostics_error_temperature_deg_c");
    if (error_it != params.end()) {
      double parsed = diagnostics_config_.error_temperature_deg_c;
      if (try_parse_double(error_it->second, parsed)) {
        diagnostics_config_.error_temperature_deg_c = parsed;
      } else {
        RCUTILS_LOG_WARN_NAMED(
          kLoggerName,
          "Invalid diagnostics_error_temperature_deg_c '%s'; keeping default",
          error_it->second.c_str());
      }
    }
  }

  diagnostics_period_ = seconds_to_duration(diagnostics_config_.period_sec);

  if (diagnostics_config_.enabled) {
    if (!diagnostics_factory_) {
      RCUTILS_LOG_WARN_NAMED(
        kLoggerName,
        "Diagnostics requested but no diagnostics factory configured; disabling diagnostics");
      diagnostics_config_.enabled = false;
    } else {
      const auto suffix = sanitize_node_suffix(info_.name);
      const std::string node_name = std::string(kDiagnosticsNodePrefix) + suffix;
      DiagnosticsCreationOptions options{
        node_name,
        info_.name.empty() ? std::string("odrive") : info_.name};
      diagnostics_ = diagnostics_factory_(options);
      if (diagnostics_) {
        diagnostics_->set_hardware_id(options.hardware_id);
        register_diagnostics_tasks();
        last_diagnostics_update_.reset();
      } else {
        diagnostics_config_.enabled = false;
      }
    }
  }

  axis_faulted_.assign(joints_.size(), false);
  idle_requested_on_fault_.assign(joints_.size(), false);

  return initialize_transport();
}

CallbackReturn ODriveHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  if (safety_config_.gate_outputs_with_lifecycle) {
    outputs_enabled_ = true;
  }

  std::fill(axis_faulted_.begin(), axis_faulted_.end(), false);
  std::fill(idle_requested_on_fault_.begin(), idle_requested_on_fault_.end(), false);

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
  if (safety_config_.gate_outputs_with_lifecycle) {
    outputs_enabled_ = false;
  }

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

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.sensors[i].name, "transport_error", &sensors_[i].transport_error));
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

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "read_error", &joints_[i].read_error));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "write_error", &joints_[i].write_error));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "telemetry_valid", &joints_[i].telemetry_valid));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "healthy", &joints_[i].healthy));
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
  const auto parse_interface_key = [](const std::string & key)
    -> std::pair<std::string, std::string> {
      const auto separator = key.find('/');
      if (separator == std::string::npos) {
        return {"", ""};
      }
      return {key.substr(0, separator), key.substr(separator + 1)};
    };

  const auto find_joint_index = [&](const std::string & joint_name) -> std::size_t {
      for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        if (info_.joints[i].name == joint_name) {
          return i;
        }
      }
      return info_.joints.size();
    };

  for (const auto & key : stop_interfaces) {
    const auto [joint_name, interface_name] = parse_interface_key(key);
    (void)interface_name;
    const auto index = find_joint_index(joint_name);
    if (index < joints_.size()) {
      joints_[index].control_level = AxisControlLevel::UNDEFINED;
    }
  }

  struct RequestedModes
  {
    bool effort{false};
    bool velocity{false};
    bool position{false};
  };

  std::vector<RequestedModes> requested_modes(joints_.size());

  for (const auto & key : start_interfaces) {
    const auto [joint_name, interface_name] = parse_interface_key(key);
    const auto index = find_joint_index(joint_name);
    if (index >= joints_.size()) {
      continue;
    }

    if (interface_name == hardware_interface::HW_IF_POSITION) {
      requested_modes[index].position = true;
    } else if (interface_name == hardware_interface::HW_IF_VELOCITY) {
      requested_modes[index].velocity = true;
    } else if (interface_name == hardware_interface::HW_IF_EFFORT) {
      requested_modes[index].effort = true;
    }
  }

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto & request = requested_modes[i];
    if (request.position) {
      joints_[i].control_level = AxisControlLevel::POSITION;
    } else if (request.velocity) {
      joints_[i].control_level = AxisControlLevel::VELOCITY;
    } else if (request.effort) {
      joints_[i].control_level = AxisControlLevel::EFFORT;
    }
  }

  return return_type::OK;
}

return_type ODriveHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> &, const std::vector<std::string> &)
{
  for (size_t i = 0; i < info_.joints.size(); i++) {
    const bool has_fault = safety_config_.mask_faulted_axes &&
      (extract_error_value(joints_[i].axis_error).has_value() ||
      extract_error_value(joints_[i].motor_error).has_value() ||
      extract_error_value(joints_[i].encoder_error).has_value() ||
      extract_error_value(joints_[i].controller_error).has_value());
    if (has_fault) {
      joints_[i].control_level = AxisControlLevel::UNDEFINED;
      continue;
    }

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

      if (i < joints_.size()) {
        joints_[i].write_error = static_cast<double>(status);
      }
      return to_io_return(status, action);
    }

    joints_[i].write_error = 0.0;
  }

  return return_type::OK;
}

void ODriveHardwareInterface::register_diagnostics_tasks()
{
  if (!diagnostics_) {
    return;
  }

  for (std::size_t i = 0; i < drives_.size(); ++i) {
    const auto task_name = std::string("ODrive/drive/") + drives_[i].label;
    diagnostics_->add_task(
      task_name,
      [this, i](diagnostic_updater::DiagnosticStatusWrapper & status) {
        populate_drive_diagnostics(status, i);
      });
  }

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto task_name = std::string("ODrive/joint/") + info_.joints[i].name;
    diagnostics_->add_task(
      task_name,
      [this, i](diagnostic_updater::DiagnosticStatusWrapper & status) {
        populate_joint_diagnostics(status, i);
      });
  }

  for (std::size_t i = 0; i < sensors_.size(); ++i) {
    const auto task_name = std::string("ODrive/sensor/") + info_.sensors[i].name;
    diagnostics_->add_task(
      task_name,
      [this, i](diagnostic_updater::DiagnosticStatusWrapper & status) {
        populate_sensor_diagnostics(status, i);
      });
  }
}

std::string ODriveHardwareInterface::serial_to_hex(std::int64_t serial_number)
{
  std::ostringstream stream;
  stream << "0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(12)
         << static_cast<std::uint64_t>(serial_number);
  return stream.str();
}

std::string ODriveHardwareInterface::join_messages(const std::vector<std::string> & parts)
{
  if (parts.empty()) {
    return {};
  }

  std::ostringstream stream;
  for (std::size_t i = 0; i < parts.size(); ++i) {
    if (i != 0) {
      stream << "; ";
    }
    stream << parts[i];
  }
  return stream.str();
}

void ODriveHardwareInterface::populate_joint_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status,
  std::size_t index) const
{
  const auto & joint_info = info_.joints[index];
  const auto & joint = joints_[index];

  bool fault_detected = false;
  bool warning_detected = false;
  std::vector<std::string> messages;

  status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Nominal");
  status.add("joint", joint_info.name);
  status.add("axis", joint.axis);
  status.add("serial", serial_to_hex(joint.serial_number));
  status.add("watchdog_enabled", joint.enable_watchdog ? "true" : "false");
  status.add("telemetry_valid", joint.telemetry_valid);
  status.add("io.read_error", joint.read_error);
  status.add("io.write_error", joint.write_error);

  const auto add_error_info = [&status, &messages, &fault_detected](
    const char * prefix,
    const std::optional<std::uint64_t> & value,
    const std::string & description) {
      const std::string bits_key = std::string(prefix) + ".error_bits";
      const std::string desc_key = std::string(prefix) + ".error_flags";
      if (value) {
        status.addf(bits_key, "0x%016" PRIX64, *value);
        if (!description.empty()) {
          status.add(desc_key, description);
          messages.emplace_back(std::string(prefix) + ": " + description);
        } else {
          status.add(desc_key, "(no description)");
          std::ostringstream stream;
          stream << std::string(prefix) << ": 0x" << std::uppercase << std::hex << *value;
          messages.emplace_back(stream.str());
        }
        fault_detected = true;
      } else {
        status.add(bits_key, "0x0");
        status.add(desc_key, "(none)");
      }
    };

  const auto axis_error_value = extract_error_value(joint.axis_error);
  const auto motor_error_value = extract_error_value(joint.motor_error);
  const auto encoder_error_value = extract_error_value(joint.encoder_error);
  const auto controller_error_value = extract_error_value(joint.controller_error);

  add_error_info(
    "axis",
    axis_error_value,
    axis_error_value ? describe_axis_error(*axis_error_value) : std::string{});

  add_error_info(
    "motor",
    motor_error_value,
    motor_error_value ? describe_motor_error(*motor_error_value) : std::string{});

  add_error_info(
    "encoder",
    encoder_error_value,
    encoder_error_value ? describe_encoder_error(*encoder_error_value) : std::string{});

  add_error_info(
    "controller",
    controller_error_value,
    controller_error_value ? describe_controller_error(*controller_error_value) : std::string{});

  const auto assess_temperature = [&](const char * label, double value) {
      if (!std::isfinite(value)) {
        status.add(label, "NaN");
        return;
      }
      status.add(label, value);
      if (value >= diagnostics_config_.error_temperature_deg_c) {
        fault_detected = true;
        std::ostringstream stream;
        stream << label << " high: " << value;
        messages.emplace_back(stream.str());
      } else if (value >= diagnostics_config_.warn_temperature_deg_c) {
        warning_detected = true;
        std::ostringstream stream;
        stream << label << " elevated: " << value;
        messages.emplace_back(stream.str());
      }
    };

  assess_temperature("temperature.fet_c", joint.fet_temperature);
  assess_temperature("temperature.motor_c", joint.motor_temperature);

  const auto summary_text = join_messages(messages);
  if (fault_detected) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      summary_text.empty() ? "Fault detected" : summary_text);
  } else if (warning_detected) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      summary_text.empty() ? "Warning" : summary_text);
  } else {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Nominal");
  }
}

void ODriveHardwareInterface::populate_drive_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status,
  std::size_t index) const
{
  const auto & drive = drives_[index];

  bool fault_detected = false;
  bool warning_detected = false;
  std::vector<std::string> messages;

  status.add("drive", drive.label);
  status.add("serial", serial_to_hex(drive.serial_number));

  status.add("can.read_error", drive.can_error_read_error);
  status.addf("can.error_bits", "0x%08" PRIX32, drive.can_error);

  if (drive.can_error_read_error != 0.0) {
    fault_detected = true;
    messages.emplace_back("can.error unavailable");
  } else if (drive.can_error != 0U) {
    warning_detected = true;
    messages.emplace_back("can.error set");
  }

  std::size_t joint_count = 0;
  std::size_t io_faulted_joints = 0;
  std::size_t faulted_joints = 0;
  for (std::size_t i = 0; i < joints_.size() && i < info_.joints.size(); ++i) {
    if (joints_[i].serial_number != drive.serial_number) {
      continue;
    }
    ++joint_count;

    const bool io_fault = (joints_[i].telemetry_valid < 0.5) ||
      (joints_[i].read_error != 0.0) || (joints_[i].write_error != 0.0);
    if (io_fault) {
      ++io_faulted_joints;
    }

    const bool axis_fault = extract_error_value(joints_[i].axis_error).has_value() ||
      extract_error_value(joints_[i].motor_error).has_value() ||
      extract_error_value(joints_[i].encoder_error).has_value() ||
      extract_error_value(joints_[i].controller_error).has_value();
    if (axis_fault) {
      ++faulted_joints;
    }
  }

  status.add("joint_count", static_cast<int>(joint_count));
  status.add("io_faulted_joints", static_cast<int>(io_faulted_joints));
  status.add("faulted_joints", static_cast<int>(faulted_joints));

  if (io_faulted_joints > 0) {
    fault_detected = true;
    messages.emplace_back("joint IO fault(s)");
  }
  if (faulted_joints > 0) {
    warning_detected = true;
    messages.emplace_back("axis fault(s)");
  }

  const auto summary_text = join_messages(messages);
  if (fault_detected) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      summary_text.empty() ? "Fault detected" : summary_text);
  } else if (warning_detected) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      summary_text.empty() ? "Warning" : summary_text);
  } else {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Nominal");
  }
}

void ODriveHardwareInterface::populate_sensor_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status,
  std::size_t index) const
{
  const auto & sensor_info = info_.sensors[index];
  const auto & sensor = sensors_[index];

  status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Nominal");
  status.add("sensor", sensor_info.name);
  status.add("serial", serial_to_hex(sensor.serial_number));

  if (std::isfinite(sensor.vbus_voltage)) {
    status.add("vbus_voltage_v", sensor.vbus_voltage);
  } else {
    status.add("vbus_voltage_v", "NaN");
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Voltage unavailable");
  }
}

void ODriveHardwareInterface::maybe_update_diagnostics()
{
  if (!diagnostics_) {
    return;
  }

  const auto now = diagnostics_->now();
  if (diagnostics_period_ <= DiagnosticsInterface::Duration::zero()) {
    diagnostics_->force_update();
    last_diagnostics_update_ = now;
    return;
  }

  if (!last_diagnostics_update_ || (now - *last_diagnostics_update_) >= diagnostics_period_) {
    diagnostics_->force_update();
    last_diagnostics_update_ = now;
  }
}

return_type ODriveHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto find_sensor_index = [&](std::int64_t serial) -> std::optional<std::size_t> {
      for (std::size_t i = 0; i < sensors_.size(); ++i) {
        if (sensors_[i].serial_number == serial) {
          return i;
        }
      }
      return std::nullopt;
    };

  // Default-unhealthy each cycle; only flip to healthy after a complete successful read.
  for (auto & joint : joints_) {
    joint.healthy = 0.0;
  }

  for (size_t i = 0; i < info_.sensors.size(); i++) {
    float vbus_voltage;

    if (const int status =
      transport_->read(sensors_[i].serial_number, odrive::VBUS_VOLTAGE, vbus_voltage);
      status != 0)
    {
      sensors_[i].transport_error = static_cast<double>(status);
      return to_io_return(status, "reading vbus voltage");
    }
    sensors_[i].vbus_voltage = vbus_voltage;
    sensors_[i].transport_error = 0.0;
  }

  // Drive-level diagnostics: read firmware-supported ODrive CAN error bitfield.
  // This is a fixed-size endpoint (unlike the JSON endpoint 0) and is safe to poll.
  for (auto & drive : drives_) {
    std::uint32_t can_error = 0;
    const int status = transport_->read(drive.serial_number, odrive::CAN__ERROR, can_error);
    if (status != 0) {
      drive.can_error_read_error = static_cast<double>(status);
      drive.can_error = 0;
      continue;
    }
    drive.can_error_read_error = 0.0;
    drive.can_error = can_error;
  }

  // Note: This firmware/API snapshot does not provide a fixed-size "top-level" drive error
  // endpoint (odrivetool also shows no root-level odrv0.error). Per-axis errors
  // (axis/motor/encoder/controller) are read below.

  for (size_t i = 0; i < info_.joints.size(); i++) {
    AxisTelemetrySample sample;
    std::string failing_stage;
    if (const int status = read_axis_telemetry(
        *transport_, joints_[i].serial_number, joints_[i].axis, joints_[i].torque_constant,
        sample,
        failing_stage);
      status != 0)
    {
      if (const auto sensor_index = find_sensor_index(joints_[i].serial_number)) {
        sensors_[*sensor_index].transport_error = static_cast<double>(status);
      }

      joints_[i].read_error = static_cast<double>(status);
      joints_[i].telemetry_valid = 0.0;
      std::string action = failing_stage.empty() ? "reading axis telemetry" :
        "reading axis telemetry (" + failing_stage + ")";
      return to_io_return(status, action);
    }

    // Independent single-source-of-truth health flag: computed from the raw values just read.
    const bool errors_clear =
      std::isfinite(sample.axis_error) && sample.axis_error == 0.0 &&
      std::isfinite(sample.motor_error) && sample.motor_error == 0.0 &&
      std::isfinite(sample.encoder_error) && sample.encoder_error == 0.0 &&
      std::isfinite(sample.controller_error) && sample.controller_error == 0.0;
    joints_[i].healthy = errors_clear ? 1.0 : 0.0;

    joints_[i].effort = sample.effort;
    joints_[i].velocity = sample.velocity;
    joints_[i].position = sample.position;
    joints_[i].axis_error = sample.axis_error;
    joints_[i].motor_error = sample.motor_error;
    joints_[i].encoder_error = sample.encoder_error;
    joints_[i].controller_error = sample.controller_error;
    joints_[i].fet_temperature = sample.fet_temperature;
    joints_[i].motor_temperature = sample.motor_temperature;
    joints_[i].read_error = 0.0;
    joints_[i].telemetry_valid = 1.0;

    if (const auto sensor_index = find_sensor_index(joints_[i].serial_number)) {
      sensors_[*sensor_index].transport_error = 0.0;
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

  maybe_update_diagnostics();

  return return_type::OK;
}

return_type ODriveHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration &)
{
  (void)time;
  const bool lifecycle_ok = !safety_config_.gate_outputs_with_lifecycle || outputs_enabled_;
  if (!lifecycle_ok) {
    return return_type::OK;
  }

  for (size_t i = 0; i < info_.joints.size(); i++) {
    const auto axis_error_value = extract_error_value(joints_[i].axis_error);
    const auto motor_error_value = extract_error_value(joints_[i].motor_error);
    const auto encoder_error_value = extract_error_value(joints_[i].encoder_error);
    const auto controller_error_value = extract_error_value(joints_[i].controller_error);

    const bool has_fault = safety_config_.mask_faulted_axes &&
      (axis_error_value.has_value() ||
      motor_error_value.has_value() ||
      encoder_error_value.has_value() ||
      controller_error_value.has_value());
    AxisCommandState command_state{
      joints_[i].command_position,
      joints_[i].command_velocity,
      joints_[i].command_effort,
      joints_[i].position,
      joints_[i].velocity,
      joints_[i].effort};

    std::string validation_error;
    const bool command_valid = validate_joint_command(
      joints_[i].command_limits, joints_[i].control_level, command_state, validation_error);

    if (!command_valid && joints_[i].control_level != AxisControlLevel::UNDEFINED) {
      RCUTILS_LOG_ERROR_NAMED(
        kLoggerName,
        "Rejected command for joint '%s': %s (falling back to zero torque)",
        info_.joints[i].name.c_str(),
        validation_error.c_str());

      // Safe fallback: hold current state (zero error) and zero torque feed-forward.
      command_state.command_position =
        std::isfinite(command_state.state_position) ? command_state.state_position : 0.0;
      command_state.command_velocity =
        std::isfinite(command_state.state_velocity) ? command_state.state_velocity : 0.0;
      command_state.command_effort = 0.0;
    }

    if (has_fault) {
      const bool can_track_fault = (i < axis_faulted_.size());
      const bool first_fault = can_track_fault && !axis_faulted_[i];

      if (first_fault) {
        RCUTILS_LOG_WARN_NAMED(
          kLoggerName,
          "Masking commands for joint '%s' due to fault(s): "
          "axis=0x%016" PRIx64 " (%s) motor=0x%016" PRIx64 " (%s) "
          "encoder=0x%016" PRIx64 " (%s) controller=0x%016" PRIx64 " (%s). "
          "Last commanded effort=%.3f (control_level=%d).",
          info_.joints[i].name.c_str(),
          axis_error_value ? static_cast<std::uint64_t>(*axis_error_value) : static_cast<std::
          uint64_t>(0),
          axis_error_value ? describe_axis_error(*axis_error_value).c_str() : "",
          motor_error_value ? static_cast<std::uint64_t>(*motor_error_value) : static_cast<std::
          uint64_t>(0),
          motor_error_value ? describe_motor_error(*motor_error_value).c_str() : "",
          encoder_error_value ?
          static_cast<std::uint64_t>(*encoder_error_value) :
          static_cast<std::uint64_t>(0),
          encoder_error_value ? describe_encoder_error(
            *encoder_error_value).c_str() : "",
          controller_error_value ?
          static_cast<std::uint64_t>(*controller_error_value) :
          static_cast<std::uint64_t>(0),
          controller_error_value ? describe_controller_error(
            *controller_error_value).c_str() : "",
          command_state.command_effort,
          static_cast<int>(joints_[i].control_level));
      }

      if (can_track_fault) {
        axis_faulted_[i] = true;
      }

      if (safety_config_.request_idle_on_axis_fault &&
        i < idle_requested_on_fault_.size() && !idle_requested_on_fault_[i])
      {
        constexpr std::int32_t requested_state = kAxisStateIdle;
        const int status = transport_->write(
          joints_[i].serial_number,
          axis_endpoint(odrive::AXIS__REQUESTED_STATE, joints_[i].axis),
          requested_state);
        if (status != 0) {
          joints_[i].write_error = static_cast<double>(status);
          return to_io_return(status, "requesting axis idle state after fault");
        }
        idle_requested_on_fault_[i] = true;
      }
      continue;
    }

    if (i < axis_faulted_.size() && axis_faulted_[i]) {
      axis_faulted_[i] = false;
      RCUTILS_LOG_INFO_NAMED(
        kLoggerName,
        "Fault cleared for joint '%s'; resuming command writes.",
        info_.joints[i].name.c_str());
    }
    if (i < idle_requested_on_fault_.size() && idle_requested_on_fault_[i]) {
      idle_requested_on_fault_[i] = false;
    }

    std::string failing_stage;
    if (const int status = write_axis_command(
        *transport_, joints_[i].serial_number, joints_[i].axis, joints_[i].control_level,
        command_state, joints_[i].enable_watchdog, failing_stage);
      status != 0)
    {
      std::string action = failing_stage.empty() ? "writing axis command" :
        "writing axis command (" + failing_stage + ")";

      joints_[i].write_error = static_cast<double>(status);
      return to_io_return(status, action);
    }

    joints_[i].write_error = 0.0;
  }

  return return_type::OK;
}
}    // namespace odrive_hardware_interface
