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
#include "odrive_hardware_interface/config_limits.hpp"
#include "odrive_hardware_interface/config_validator.hpp"
#include "odrive_hardware_interface/error_monitoring.hpp"
#include "odrive_hardware_interface/odrive_usb.hpp"
#include "odrive_hardware_interface/transport_error.hpp"
#include "rcutils/logging_macros.h"

namespace odrive_hardware_interface
{

namespace
{
constexpr const char kLoggerName[] = "ODriveHardwareInterface";
constexpr const char * kDiagnosticsNodePrefix = "odrive_diagnostics_";
constexpr std::uint32_t kAxisErrorWatchdogTimerExpired = 0x00000800U;

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

// Retry wrapper for USB operations
// NOTE: Uses FIXED retry count and delay for deterministic real-time behavior.
// Do not make this adaptive - the control loop needs predictable worst-case timing.
// If retries are exhausted, the error surfaces immediately for proper fault handling.
template<typename T>
int read_with_retry(
  ODriveTransport & transport,
  std::int64_t serial,
  std::int16_t endpoint,
  T & value,
  unsigned int max_attempts,
  std::chrono::milliseconds retry_delay,
  std::size_t & retry_count)
{
  int last_error = 0;

  for (unsigned int attempt = 0; attempt < max_attempts; ++attempt) {
    int status = transport.read(serial, endpoint, value);

    if (status == 0) {
      if (attempt > 0) {
        retry_count += attempt;
      }
      return 0;  // Success
    }

    last_error = status;
    TransportError err = errno_to_transport_error(status);

    // Don't retry fatal errors
    if (is_fatal(err)) {
      break;
    }

    // Retry on transient errors
    if (is_retryable(err) && attempt < max_attempts - 1) {
      std::this_thread::sleep_for(retry_delay);
    } else {
      break;
    }
  }

  return last_error;
}

template<typename T>
int write_with_retry(
  ODriveTransport & transport,
  std::int64_t serial,
  std::int16_t endpoint,
  const T & value,
  unsigned int max_attempts,
  std::chrono::milliseconds retry_delay,
  std::size_t & retry_count)
{
  int last_error = 0;

  for (unsigned int attempt = 0; attempt < max_attempts; ++attempt) {
    int status = transport.write(serial, endpoint, value);

    if (status == 0) {
      if (attempt > 0) {
        retry_count += attempt;
      }
      return 0;
    }

    last_error = status;
    TransportError err = errno_to_transport_error(status);

    if (is_fatal(err)) {
      break;
    }

    if (is_retryable(err) && attempt < max_attempts - 1) {
      std::this_thread::sleep_for(retry_delay);
    } else {
      break;
    }
  }

  return last_error;
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

void ODriveHardwareInterface::hold_joint_state(JointContext & joint)
{
  if (!std::isfinite(joint.last_valid_position)) {
    joint.last_valid_position = 0.0;
  }
  joint.position = joint.last_valid_position;
  joint.velocity = 0.0;
  joint.effort = 0.0;
}

void ODriveHardwareInterface::reset_runtime_state()
{
  outputs_enabled_ = false;

  for (auto & sensor : sensors_) {
    sensor.vbus_voltage = std::numeric_limits<double>::quiet_NaN();
    sensor.transport_error = 0.0;
  }

  for (auto & joint : joints_) {
    // Keep TF-safe kinematic state finite across deactivation/recovery.
    // Freeze to last known finite position; zero velocity/effort.
    if (!std::isfinite(joint.position)) {
      joint.position = 0.0;
    }
    joint.velocity = 0.0;
    joint.effort = 0.0;

    // Reset command values to safe finite defaults.
    joint.command_position = joint.position;
    joint.command_velocity = 0.0;
    joint.command_effort = 0.0;

    joint.last_command_position = joint.command_position;
    joint.last_command_velocity = joint.command_velocity;
    joint.last_command_effort = joint.command_effort;

    joint.last_valid_position = joint.position;
    joint.last_valid_velocity = joint.velocity;

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
  joint_claimed_.assign(joints_.size(), false);
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

  // Propagate runtime-configured timeout (if supported by transport).
  transport_->set_timeout_ms(runtime_config_.usb_timeout_ms);

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

  for (std::size_t i = 0; i < joints_.size(); i++) {
    auto & joint_context = joints_[i];
    const auto & joint_config = hardware_config_.joints[i];

    float torque_constant = std::numeric_limits<float>::quiet_NaN();
    const int read_status = read_with_retry(
      *transport_,
      joint_context.serial_number,
      axis_endpoint(odrive::AXIS__MOTOR__CONFIG__TORQUE_CONSTANT, joint_context.axis),
      torque_constant,
      runtime_config_.usb_read_retries,
      std::chrono::milliseconds(runtime_config_.usb_retry_delay_ms),
      cycle_stats_.usb_read_retries_total);
    if (read_status != 0) {
      return to_callback_return(read_status, "reading motor torque constant");
    }
    joint_context.torque_constant = torque_constant;

    if (joint_context.enable_watchdog) {
      const int write_timeout_status = write_with_retry(
        *transport_,
        joint_context.serial_number,
        axis_endpoint(odrive::AXIS__CONFIG__WATCHDOG_TIMEOUT, joint_context.axis),
        static_cast<float>(joint_config.watchdog_timeout),
        runtime_config_.usb_write_retries,
        std::chrono::milliseconds(runtime_config_.usb_retry_delay_ms),
        cycle_stats_.usb_write_retries_total);
      if (write_timeout_status != 0) {
        return to_callback_return(write_timeout_status, "configuring watchdog timeout");
      }
    }

    // Do not arm/enable the watchdog during initialization. It is enabled on command interface
    // claim (perform_command_mode_switch) to avoid false positives before a controller commands.
    const int write_enable_status = write_with_retry(
      *transport_,
      joint_context.serial_number,
      axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, joint_context.axis),
      static_cast<bool>(false),
      runtime_config_.usb_write_retries,
      std::chrono::milliseconds(runtime_config_.usb_retry_delay_ms),
      cycle_stats_.usb_write_retries_total);
    if (write_enable_status != 0) {
      return to_callback_return(write_enable_status, "disabling watchdog during initialization");
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
    joint.invert_axis = joint_config.invert_axis;
    joint.enable_watchdog = joint_config.enable_watchdog;
    joint.command_limits = joint_config.command_limits;
    joint.feedback_limits = joint_config.feedback_limits;
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
  runtime_config_ = RuntimeConfig{};
  cycle_stats_ = CycleStats{};

  {
    auto params = info_.hardware_parameters;

    // Parse runtime configuration parameters
    auto usb_timeout_it = params.find("usb_timeout_ms");
    if (usb_timeout_it != params.end()) {
      ConfigValidator::parse_unsigned_int(
        kLoggerName,
        "usb_timeout_ms",
        usb_timeout_it->second,
        config_limits::MIN_USB_TIMEOUT_MS,
        config_limits::MAX_USB_TIMEOUT_MS,
        config_limits::DEFAULT_USB_TIMEOUT_MS,
        runtime_config_.usb_timeout_ms);
    }

    auto log_throttle_it = params.find("log_throttle_ms");
    if (log_throttle_it != params.end()) {
      ConfigValidator::parse_unsigned_int(
        kLoggerName,
        "log_throttle_ms",
        log_throttle_it->second,
        config_limits::MIN_LOG_THROTTLE_MS,
        config_limits::MAX_LOG_THROTTLE_MS,
        config_limits::DEFAULT_LOG_THROTTLE_MS,
        runtime_config_.log_throttle_ms);
    }

    auto max_read_it = params.find("max_read_cycle_time_sec");
    if (max_read_it != params.end()) {
      double parsed = runtime_config_.max_read_cycle_time_sec;
      if (try_parse_double(max_read_it->second, parsed) && parsed > 0.0) {
        runtime_config_.max_read_cycle_time_sec = parsed;
      } else {
        RCUTILS_LOG_WARN_NAMED(
          kLoggerName,
          "Invalid max_read_cycle_time_sec '%s'; keeping default",
          max_read_it->second.c_str());
      }
    }

    auto max_write_it = params.find("max_write_cycle_time_sec");
    if (max_write_it != params.end()) {
      double parsed = runtime_config_.max_write_cycle_time_sec;
      if (try_parse_double(max_write_it->second, parsed) && parsed > 0.0) {
        runtime_config_.max_write_cycle_time_sec = parsed;
      } else {
        RCUTILS_LOG_WARN_NAMED(
          kLoggerName,
          "Invalid max_write_cycle_time_sec '%s'; keeping default",
          max_write_it->second.c_str());
      }
    }

    auto deadline_warnings_it = params.find("enable_deadline_warnings");
    if (deadline_warnings_it != params.end()) {
      bool parsed = runtime_config_.enable_deadline_warnings;
      if (try_parse_bool(deadline_warnings_it->second, parsed)) {
        runtime_config_.enable_deadline_warnings = parsed;
      } else {
        RCUTILS_LOG_WARN_NAMED(
          kLoggerName,
          "Invalid enable_deadline_warnings '%s'; keeping default",
          deadline_warnings_it->second.c_str());
      }
    }

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
        if (parsed_period < config_limits::MIN_DIAGNOSTIC_PERIOD_SEC) {
          RCUTILS_LOG_WARN_NAMED(
            kLoggerName,
            "Diagnostics period %.3fs is below minimum %.3fs; "
            "clamping to minimum to protect real-time performance",
            parsed_period, config_limits::MIN_DIAGNOSTIC_PERIOD_SEC);
          diagnostics_config_.period_sec = config_limits::MIN_DIAGNOSTIC_PERIOD_SEC;
        } else {
          diagnostics_config_.period_sec = parsed_period;
        }
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

  // Initialize all runtime state for the newly parsed joints/sensors.
  // This keeps exported joint states TF-safe (finite) even before the first read().
  reset_runtime_state();

  return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(
    kLoggerName,
    "Activating ODrive hardware: %zu joints, %zu sensors, %zu drives",
    joints_.size(), sensors_.size(), drives_.size());

  for (std::size_t i = 0; i < joints_.size() && i < hardware_config_.joints.size(); ++i) {
    RCUTILS_LOG_INFO_NAMED(
      kLoggerName,
      "  Joint[%zu] '%s': serial=0x%012" PRIX64 " axis=%d watchdog=%s timeout=%.3fs",
      i, info_.joints[i].name.c_str(),
      static_cast<std::uint64_t>(joints_[i].serial_number),
      joints_[i].axis,
      joints_[i].enable_watchdog ? "enabled" : "disabled",
      hardware_config_.joints[i].watchdog_timeout);
  }

  // Attempt hardware connection.
  // If this fails, keep the component ACTIVE but do not enable outputs.
  // read()/write() already handle transport errors gracefully and will report unhealthy state.
  const auto init_result = initialize_transport();
  if (init_result != CallbackReturn::SUCCESS) {
    if (safety_config_.gate_outputs_with_lifecycle) {
      outputs_enabled_.store(false, std::memory_order_release);
    }
    RCUTILS_LOG_ERROR_NAMED(
      kLoggerName,
      "Failed to initialize ODrive transport during activation. "
      "Continuing without hardware (outputs disabled). "
      "Check USB connections and reactivate to retry.");
    return CallbackReturn::SUCCESS;
  }

  if (safety_config_.gate_outputs_with_lifecycle) {
    // Atomic store with release semantics ensures all prior initialization is visible
    // to threads that read outputs_enabled_ with acquire semantics
    outputs_enabled_.store(true, std::memory_order_release);
  }

  std::fill(axis_faulted_.begin(), axis_faulted_.end(), false);
  std::fill(idle_requested_on_fault_.begin(), idle_requested_on_fault_.end(), false);
  std::fill(joint_claimed_.begin(), joint_claimed_.end(), false);

  for (auto & joint : joints_) {
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
    // Atomic store with release semantics ensures deactivation is visible
    // to write() thread before it checks the flag
    outputs_enabled_.store(false, std::memory_order_release);
  }

  // Ensure transport_ is not reset while read()/write() is active.
  std::unique_lock<std::shared_mutex> lock(state_mutex_);

  if (!transport_) {
    return CallbackReturn::SUCCESS;
  }

  constexpr std::int32_t requested_state = kAxisStateIdle;
  for (std::size_t i = 0; i < joints_.size() && i < info_.joints.size(); ++i) {
    const auto & joint = joints_[i];
    const int status = transport_->write(
      joint.serial_number,
      axis_endpoint(odrive::AXIS__REQUESTED_STATE, joint.axis),
      requested_state);
    if (status != 0) {
      // Deactivation should be best-effort: if the transport is already gone,
      // we still want ros2_control to be able to transition cleanly.
      RCUTILS_LOG_WARN_NAMED(
        kLoggerName,
        "Transport error (%d) while requesting axis idle state for joint[%zu]='%s' "
        "(serial 0x%016" PRIx64 " axis %d). Continuing deactivation.",
        status,
        i,
        info_.joints[i].name.c_str(),
        static_cast<std::uint64_t>(joint.serial_number),
        joint.axis);

      // Drop the transport so subsequent read()/write() calls take the
      // already-existing 'no transport' graceful path.
      transport_.reset();
      break;
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
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.sensors.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.sensors[i].name, "vbus_voltage", &sensors_[i].vbus_voltage));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.sensors[i].name, "transport_error", &sensors_[i].transport_error));
  }

  for (std::size_t i = 0; i < info_.joints.size(); i++) {
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

    // Rate limiter state interfaces
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "position_rate_limited", &joints_[i].position_rate_limited));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "velocity_rate_limited", &joints_[i].velocity_rate_limited));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "effort_rate_limited", &joints_[i].effort_rate_limited));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ODriveHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
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
  if (!transport_) {
    return return_type::OK;
  }

  // Ensure claimed-state tracking is sized correctly (can be reset on (re)configure).
  if (joint_claimed_.size() != joints_.size()) {
    joint_claimed_.assign(joints_.size(), false);
  }

  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    const bool was_claimed = (i < joint_claimed_.size()) ? joint_claimed_[i] : false;
    const bool is_claimed = (joints_[i].control_level != AxisControlLevel::UNDEFINED);

    // Claim transition: enable+feed watchdog once (if configured), then clear ONLY the
    // pure watchdog-timer-expired axis fault so controllers can reclaim without a full
    // deactivate/activate.
    if (is_claimed && !was_claimed) {
      if (joints_[i].enable_watchdog) {
        const int enable_status = transport_->write(
          joints_[i].serial_number,
          axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, joints_[i].axis),
          static_cast<bool>(true));
        if (enable_status != 0) {
          joints_[i].write_error = static_cast<double>(enable_status);
          return to_io_return(enable_status, "enabling watchdog on interface claim");
        }

        const int feed_status = transport_->call(
          joints_[i].serial_number,
          axis_endpoint(odrive::AXIS__WATCHDOG_FEED, joints_[i].axis));
        if (feed_status != 0) {
          joints_[i].write_error = static_cast<double>(feed_status);
          return to_io_return(feed_status, "feeding watchdog on interface claim");
        }
      }

      std::int32_t axis_error_raw = 0;
      const int read_error_status = transport_->read(
        joints_[i].serial_number,
        axis_endpoint(odrive::AXIS__ERROR, joints_[i].axis),
        axis_error_raw);
      if (read_error_status != 0) {
        joints_[i].write_error = static_cast<double>(read_error_status);
        return to_io_return(read_error_status, "reading axis error on interface claim");
      }

      const std::uint32_t axis_error = static_cast<std::uint32_t>(axis_error_raw);
      joints_[i].axis_error = static_cast<double>(axis_error);

      if (axis_error == kAxisErrorWatchdogTimerExpired) {
        const int clear_status = transport_->call(
          joints_[i].serial_number,
          axis_endpoint(odrive::AXIS__CLEAR_ERRORS, joints_[i].axis));
        if (clear_status != 0) {
          joints_[i].write_error = static_cast<double>(clear_status);
          return to_io_return(clear_status, "clearing pure watchdog expiry on interface claim");
        }

        // Reflect the clear locally so fault-masking doesn't block the immediate mode switch.
        joints_[i].axis_error = 0.0;
      }
    }

    // Unclaim transition: best-effort request IDLE and disable watchdog so it won't expire
    // while no controller is commanding.
    if (!is_claimed && was_claimed) {
      constexpr std::int32_t requested_state = kAxisStateIdle;
      const int idle_status = transport_->write(
        joints_[i].serial_number,
        axis_endpoint(odrive::AXIS__REQUESTED_STATE, joints_[i].axis),
        requested_state);
      if (idle_status != 0) {
        RCUTILS_LOG_WARN_NAMED(
          kLoggerName,
          "Transport error (%d) requesting axis idle state on interface unclaim "
          "for joint[%zu]='%s'; continuing",
          idle_status,
          i,
          info_.joints[i].name.c_str());
      }

      const int disable_status = transport_->write(
        joints_[i].serial_number,
        axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, joints_[i].axis),
        static_cast<bool>(false));
      if (disable_status != 0) {
        RCUTILS_LOG_WARN_NAMED(
          kLoggerName,
          "Transport error (%d) disabling watchdog on interface unclaim "
          "for joint[%zu]='%s'; continuing",
          disable_status,
          i,
          info_.joints[i].name.c_str());
      }
    }

    const bool has_fault = safety_config_.mask_faulted_axes &&
      (extract_error_value(joints_[i].axis_error).has_value() ||
      extract_error_value(joints_[i].motor_error).has_value() ||
      extract_error_value(joints_[i].encoder_error).has_value() ||
      extract_error_value(joints_[i].controller_error).has_value());
    if (has_fault) {
      joints_[i].control_level = AxisControlLevel::UNDEFINED;
      if (i < joint_claimed_.size()) {
        joint_claimed_[i] = false;
      }
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

    if (i < joint_claimed_.size()) {
      joint_claimed_[i] = (joints_[i].control_level != AxisControlLevel::UNDEFINED);
    }
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
  // Use shared lock - diagnostics read-only access allows concurrency with other readers
  std::shared_lock<std::shared_mutex> lock(state_mutex_);

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

  // Add rate limiter diagnostics
  status.add("rate_limit.position", joint.position_rate_limited > 0.5 ? "active" : "inactive");
  status.add("rate_limit.velocity", joint.velocity_rate_limited > 0.5 ? "active" : "inactive");
  status.add("rate_limit.effort", joint.effort_rate_limited > 0.5 ? "active" : "inactive");

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
  // Use shared lock - diagnostics read-only access allows concurrency with other readers
  std::shared_lock<std::shared_mutex> lock(state_mutex_);

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

  // Add performance statistics
  status.add("stats.read_cycles", static_cast<int>(cycle_stats_.read_cycles));
  status.add("stats.write_cycles", static_cast<int>(cycle_stats_.write_cycles));
  status.add("stats.read_deadline_misses", static_cast<int>(cycle_stats_.read_deadline_misses));
  status.add("stats.write_deadline_misses", static_cast<int>(cycle_stats_.write_deadline_misses));
  status.addf("stats.max_read_time_ms", "%.3f", cycle_stats_.max_read_cycle_time_sec * 1000.0);
  status.addf("stats.max_write_time_ms", "%.3f", cycle_stats_.max_write_cycle_time_sec * 1000.0);
  status.add("stats.usb_read_retries", static_cast<int>(cycle_stats_.usb_read_retries_total));
  status.add("stats.usb_write_retries", static_cast<int>(cycle_stats_.usb_write_retries_total));
  status.add(
    "stats.validation_failures",
    static_cast<int>(cycle_stats_.command_validation_failures));
  status.add("stats.rate_limit_events", static_cast<int>(cycle_stats_.rate_limit_events));

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
  // Use shared lock - diagnostics read-only access allows concurrency with other readers
  std::shared_lock<std::shared_mutex> lock(state_mutex_);

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
  auto start_time = std::chrono::steady_clock::now();

  const auto is_fail_fast_transport_error = [](int status) -> bool {
      return status == LIBUSB_ERROR_TIMEOUT || status == LIBUSB_ERROR_NO_DEVICE;
    };

  const auto mark_all_unhealthy_and_hold = [this](int status, const char * context) {
      RCUTILS_LOG_ERROR_THROTTLE_NAMED(
        RCUTILS_STEADY_TIME, runtime_config_.log_throttle_ms, kLoggerName,
        "Fail-fast transport error (%d) during %s; marking all joints unhealthy",
        status, context);
      for (auto & joint : joints_) {
        joint.healthy = 0.0;
        joint.telemetry_valid = 0.0;
        hold_joint_state(joint);
      }
      for (auto & sensor : sensors_) {
        sensor.vbus_voltage = std::numeric_limits<double>::quiet_NaN();
      }
    };

  // Use shared lock - read() only modifies state_, not command interfaces
  // This allows diagnostics callbacks to run concurrently during read()
  std::shared_lock<std::shared_mutex> lock(state_mutex_);

  // If transport lost (disconnected during operation), set all joints unhealthy and return.
  // Recovery requires deactivate+activate cycle.
  if (!transport_) {
    for (auto & joint : joints_) {
      joint.healthy = 0.0;
      joint.telemetry_valid = 0.0;
      hold_joint_state(joint);
    }
    for (auto & sensor : sensors_) {
      sensor.vbus_voltage = std::numeric_limits<double>::quiet_NaN();
    }
    RCUTILS_LOG_WARN_THROTTLE_NAMED(
      RCUTILS_STEADY_TIME, runtime_config_.log_throttle_ms, kLoggerName,
      "Transport unavailable during read(). All joints marked unhealthy. "
      "Deactivate and reactivate hardware to reconnect.");
    return return_type::OK;
  }

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
      sensors_[i].vbus_voltage = std::numeric_limits<double>::quiet_NaN();

      if (is_fail_fast_transport_error(status)) {
        mark_all_unhealthy_and_hold(status, "reading vbus voltage");
        return return_type::OK;
      }

      RCUTILS_LOG_WARN_THROTTLE_NAMED(
        RCUTILS_STEADY_TIME, runtime_config_.log_throttle_ms, kLoggerName,
        "Transport error (%d) reading vbus voltage for sensor[%zu] "
        "(serial 0x%" PRIx64 "); continuing",
        status, i, static_cast<std::uint64_t>(sensors_[i].serial_number));
      continue;
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

      if (is_fail_fast_transport_error(status)) {
        mark_all_unhealthy_and_hold(status, "reading CAN error");
        return return_type::OK;
      }

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
      if (is_fail_fast_transport_error(status)) {
        mark_all_unhealthy_and_hold(status, "reading axis telemetry");
        return return_type::OK;
      }

      if (const auto sensor_index = find_sensor_index(joints_[i].serial_number)) {
        sensors_[*sensor_index].transport_error = static_cast<double>(status);
      }

      joints_[i].read_error = static_cast<double>(status);
      joints_[i].telemetry_valid = 0.0;
      joints_[i].healthy = 0.0;
      hold_joint_state(joints_[i]);
      std::string action = failing_stage.empty() ? "reading axis telemetry" :
        "reading axis telemetry (" + failing_stage + ")";
      RCUTILS_LOG_WARN_THROTTLE_NAMED(
        RCUTILS_STEADY_TIME, config_limits::DEFAULT_LOG_THROTTLE_MS, kLoggerName,
        "Transport error (%d) %s for joint[%zu]='%s' (serial 0x%016" PRIx64 " axis %d); "
        "continuing. Last known: pos=%.3f vel=%.3f eff=%.3f",
        status,
        action.c_str(), i, info_.joints[i].name.c_str(),
        static_cast<std::uint64_t>(joints_[i].serial_number), joints_[i].axis,
        joints_[i].position, joints_[i].velocity, joints_[i].effort);
      continue;
    }

    // Feedback validation (sanity checks)
    bool feedback_valid = true;
    std::string validation_error;

    // Check for non-finite values
    if (!std::isfinite(sample.position) || !std::isfinite(sample.velocity) ||
      !std::isfinite(sample.effort))
    {
      feedback_valid = false;
      validation_error = "Non-finite feedback values";
    }

    // Check believable velocity limit
    if (feedback_valid && joints_[i].feedback_limits.max_believable_velocity &&
      std::abs(sample.velocity) > *joints_[i].feedback_limits.max_believable_velocity)
    {
      feedback_valid = false;
      validation_error = "Velocity " + std::to_string(sample.velocity) +
        " exceeds believable limit " +
        std::to_string(*joints_[i].feedback_limits.max_believable_velocity);
    }

    // Check believable effort limit
    if (feedback_valid && joints_[i].feedback_limits.max_believable_effort &&
      std::abs(sample.effort) > *joints_[i].feedback_limits.max_believable_effort)
    {
      feedback_valid = false;
      validation_error = "Effort " + std::to_string(sample.effort) +
        " exceeds believable limit " +
        std::to_string(*joints_[i].feedback_limits.max_believable_effort);
    }

    // Check position discontinuity
    if (feedback_valid && std::isfinite(joints_[i].last_valid_position) &&
      joints_[i].feedback_limits.max_position_discontinuity)
    {
      double position_jump = std::abs(sample.position - joints_[i].last_valid_position);
      if (position_jump > *joints_[i].feedback_limits.max_position_discontinuity) {
        feedback_valid = false;
        validation_error = "Position discontinuity " + std::to_string(position_jump) +
          " exceeds limit " +
          std::to_string(*joints_[i].feedback_limits.max_position_discontinuity);
      }
    }

    if (!feedback_valid) {
      joints_[i].read_error = -1.0;  // Validation error
      joints_[i].telemetry_valid = 0.0;
      joints_[i].healthy = 0.0;
      hold_joint_state(joints_[i]);
      RCUTILS_LOG_WARN_THROTTLE_NAMED(
        RCUTILS_STEADY_TIME, runtime_config_.log_throttle_ms, kLoggerName,
        "Feedback validation failed for joint[%zu]='%s': %s. "
        "Keeping last valid state: pos=%.3f vel=%.3f eff=%.3f",
        i, info_.joints[i].name.c_str(), validation_error.c_str(),
        joints_[i].position, joints_[i].velocity, joints_[i].effort);
      continue;
    }

    // Independent single-source-of-truth health flag: computed from the raw values just read.
    const bool errors_clear =
      std::isfinite(sample.axis_error) && sample.axis_error == 0.0 &&
      std::isfinite(sample.motor_error) && sample.motor_error == 0.0 &&
      std::isfinite(sample.encoder_error) && sample.encoder_error == 0.0 &&
      std::isfinite(sample.controller_error) && sample.controller_error == 0.0;
    joints_[i].healthy = errors_clear ? 1.0 : 0.0;

    // Apply axis inversion if configured (for reversed motor wiring)
    const double sign = joints_[i].invert_axis ? -1.0 : 1.0;
    if (errors_clear) {
      joints_[i].effort = sample.effort * sign;
      joints_[i].velocity = sample.velocity * sign;
      joints_[i].position = sample.position * sign;
    } else {
      // Axis is unhealthy (error registers non-zero): freeze position and zero velocity/effort.
      hold_joint_state(joints_[i]);
    }
    joints_[i].axis_error = sample.axis_error;
    joints_[i].motor_error = sample.motor_error;
    joints_[i].encoder_error = sample.encoder_error;
    joints_[i].controller_error = sample.controller_error;
    joints_[i].fet_temperature = sample.fet_temperature;
    joints_[i].motor_temperature = sample.motor_temperature;
    joints_[i].read_error = 0.0;
    joints_[i].telemetry_valid = 1.0;

    // Only advance the "last valid" values when we're actually accepting telemetry.
    if (errors_clear) {
      joints_[i].last_valid_position = joints_[i].position;
      joints_[i].last_valid_velocity = joints_[i].velocity;
    }

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

  // Cycle statistics and deadline monitoring
  ++cycle_stats_.read_cycles;
  auto cycle_duration = std::chrono::steady_clock::now() - start_time;
  auto duration_sec = std::chrono::duration<double>(cycle_duration).count();

  if (duration_sec > cycle_stats_.max_read_cycle_time_sec) {
    cycle_stats_.max_read_cycle_time_sec = duration_sec;
  }

  if (runtime_config_.enable_deadline_warnings) {
    if (duration_sec > runtime_config_.max_read_cycle_time_sec) {
      ++cycle_stats_.read_deadline_misses;
      RCUTILS_LOG_WARN_THROTTLE_NAMED(
        RCUTILS_STEADY_TIME, runtime_config_.log_throttle_ms, kLoggerName,
        "read() cycle time %.3fms exceeded limit %.3fms (miss %zu of %zu cycles)",
        duration_sec * 1000.0,
        runtime_config_.max_read_cycle_time_sec * 1000.0,
        cycle_stats_.read_deadline_misses,
        cycle_stats_.read_cycles);
    }
  }

  return return_type::OK;
}

return_type ODriveHardwareInterface::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  auto start_time = std::chrono::steady_clock::now();

  const auto is_fail_fast_transport_error = [](int status) -> bool {
      return status == LIBUSB_ERROR_TIMEOUT || status == LIBUSB_ERROR_NO_DEVICE;
    };

  // Use unique lock - write() modifies state and must have exclusive access
  // This blocks all readers (diagnostics) and other writers during command transmission
  std::unique_lock<std::shared_mutex> lock(state_mutex_);
  (void)time;

  // Calculate time delta for rate limiting
  double dt = period.seconds();
  if (dt <= 0.0 || !std::isfinite(dt)) {
    dt = 0.001;  // Assume 1ms if period invalid
  }

  // If transport lost, skip all writes. Joints are already marked unhealthy in read().
  if (!transport_) {
    RCUTILS_LOG_WARN_THROTTLE_NAMED(
      RCUTILS_STEADY_TIME, runtime_config_.log_throttle_ms, kLoggerName,
      "Transport unavailable during write(). Skipping all command writes. "
      "Deactivate and reactivate hardware to reconnect.");
    return return_type::OK;
  }

  // Lock-free atomic read of lifecycle state
  const bool lifecycle_ok = !safety_config_.gate_outputs_with_lifecycle ||
    outputs_enabled_.load(std::memory_order_acquire);
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

    // Apply rate limiting before validation
    std::ostringstream rate_limit_msg;
    bool was_rate_limited = false;

    // Reset rate limiter state flags
    joints_[i].position_rate_limited = 0.0;
    joints_[i].velocity_rate_limited = 0.0;
    joints_[i].effort_rate_limited = 0.0;

    if (joints_[i].control_level == AxisControlLevel::POSITION) {
      if (!apply_rate_limiting(
          "Position",
          command_state.command_position,
          joints_[i].last_command_position,
          joints_[i].command_limits.max_position_rate,
          dt,
          rate_limit_msg))
      {
        was_rate_limited = true;
        joints_[i].position_rate_limited = 1.0;
        ++cycle_stats_.rate_limit_events;
      }
    }

    if (joints_[i].control_level == AxisControlLevel::POSITION ||
      joints_[i].control_level == AxisControlLevel::VELOCITY)
    {
      if (!apply_rate_limiting(
          "Velocity",
          command_state.command_velocity,
          joints_[i].last_command_velocity,
          joints_[i].command_limits.max_velocity_rate,
          dt,
          rate_limit_msg))
      {
        was_rate_limited = true;
        joints_[i].velocity_rate_limited = 1.0;
        ++cycle_stats_.rate_limit_events;
      }
    }

    if (!apply_rate_limiting(
        "Effort",
        command_state.command_effort,
        joints_[i].last_command_effort,
        joints_[i].command_limits.max_effort_rate,
        dt,
        rate_limit_msg))
    {
      was_rate_limited = true;
      joints_[i].effort_rate_limited = 1.0;
      ++cycle_stats_.rate_limit_events;
    }

    if (was_rate_limited) {
      RCUTILS_LOG_WARN_THROTTLE_NAMED(
        RCUTILS_STEADY_TIME, runtime_config_.log_throttle_ms, kLoggerName,
        "Rate limited joint '%s': %s",
        info_.joints[i].name.c_str(),
        rate_limit_msg.str().c_str());
    }

    std::string validation_error;
    const bool command_valid = validate_joint_command(
      joints_[i].command_limits, joints_[i].control_level, command_state, validation_error);

    if (!command_valid && joints_[i].control_level != AxisControlLevel::UNDEFINED) {
      ++cycle_stats_.command_validation_failures;
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

          if (is_fail_fast_transport_error(status)) {
            transport_.reset();
            RCUTILS_LOG_ERROR_THROTTLE_NAMED(
              RCUTILS_STEADY_TIME, runtime_config_.log_throttle_ms, kLoggerName,
              "Fail-fast transport error (%d) requesting axis idle state; transport dropped",
              status);
            return return_type::OK;
          }

          RCUTILS_LOG_WARN_THROTTLE_NAMED(
            RCUTILS_STEADY_TIME, runtime_config_.log_throttle_ms, kLoggerName,
            "Transport error (%d) requesting axis idle state after fault "
            "for joint[%zu]='%s'; continuing",
            status, i, info_.joints[i].name.c_str());
        } else {
          idle_requested_on_fault_[i] = true;
        }
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

    // Write commands and feed watchdog (if enabled).
    // Note: Watchdog feed is intentionally tied to command writes to detect
    // control loop failures. If commands stop (lifecycle gating, fault masking),
    // the watchdog will expire and safely halt the motor.

    // Apply axis inversion if configured (for reversed motor wiring)
    if (joints_[i].invert_axis) {
      command_state.command_position *= -1.0;
      command_state.command_velocity *= -1.0;
      command_state.command_effort *= -1.0;
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

      if (is_fail_fast_transport_error(status)) {
        transport_.reset();
        RCUTILS_LOG_ERROR_THROTTLE_NAMED(
          RCUTILS_STEADY_TIME, runtime_config_.log_throttle_ms, kLoggerName,
          "Fail-fast transport error (%d) %s; transport dropped",
          status, action.c_str());
        return return_type::OK;
      }

      RCUTILS_LOG_WARN_THROTTLE_NAMED(
        RCUTILS_STEADY_TIME, config_limits::DEFAULT_LOG_THROTTLE_MS, kLoggerName,
        "Transport error (%d) %s for joint[%zu]='%s' (serial 0x%016" PRIx64 " axis %d); "
        "continuing. Command: pos=%.3f vel=%.3f eff=%.3f level=%d",
        status,
        action.c_str(), i, info_.joints[i].name.c_str(),
        static_cast<std::uint64_t>(joints_[i].serial_number), joints_[i].axis,
        command_state.command_position, command_state.command_velocity,
        command_state.command_effort, static_cast<int>(joints_[i].control_level));
      continue;
    }

    joints_[i].write_error = 0.0;

    // Update last command values for next cycle's rate limiting
    joints_[i].last_command_position = command_state.command_position;
    joints_[i].last_command_velocity = command_state.command_velocity;
    joints_[i].last_command_effort = command_state.command_effort;
  }

  // Cycle statistics and deadline monitoring
  ++cycle_stats_.write_cycles;
  auto cycle_duration = std::chrono::steady_clock::now() - start_time;
  auto duration_sec = std::chrono::duration<double>(cycle_duration).count();

  if (duration_sec > cycle_stats_.max_write_cycle_time_sec) {
    cycle_stats_.max_write_cycle_time_sec = duration_sec;
  }

  if (runtime_config_.enable_deadline_warnings) {
    if (duration_sec > runtime_config_.max_write_cycle_time_sec) {
      ++cycle_stats_.write_deadline_misses;
      RCUTILS_LOG_WARN_THROTTLE_NAMED(
        RCUTILS_STEADY_TIME, runtime_config_.log_throttle_ms, kLoggerName,
        "write() cycle time %.3fms exceeded limit %.3fms (miss %zu of %zu cycles)",
        duration_sec * 1000.0,
        runtime_config_.max_write_cycle_time_sec * 1000.0,
        cycle_stats_.write_deadline_misses,
        cycle_stats_.write_cycles);
    }
  }

  return return_type::OK;
}
}    // namespace odrive_hardware_interface
