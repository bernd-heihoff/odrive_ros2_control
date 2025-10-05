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

#include <array>
#include <cmath>
#include <optional>
#include <sstream>
#include <utility>
#include <vector>

#include "odrive_hardware_interface/axis_control.hpp"
#include "odrive_hardware_interface/axis_telemetry.hpp"
#include "odrive_hardware_interface/axis_utils.hpp"
#include "odrive_hardware_interface/odrive_usb.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace odrive_hardware_interface
{

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

namespace
{
constexpr const char kLoggerName[] = "ODriveHardwareInterface";

constexpr std::array<std::pair<std::int32_t, const char *>, 32> kAxisErrorTable{{
  {0x00000001, "invalid_state"},
  {0x00000002, "dc_bus_under_voltage"},
  {0x00000004, "dc_bus_over_voltage"},
  {0x00000008, "current_limit_violation"},
  {0x00000010, "motor_disarmed"},
  {0x00000020, "motor_failed"},
  {0x00000040, "sensorless_estimator_failed"},
  {0x00000080, "encoder_failed"},
  {0x00000100, "controller_failed"},
  {0x00000200, "watchdog_timer_expired"},
  {0x00000400, "min_endstop_pressed"},
  {0x00000800, "max_endstop_pressed"},
  {0x00001000, "estop_requested"},
  {0x00002000, "homing_without_endstop"},
  {0x00004000, "over_temp"},
  {0x00008000, "unknown_position"},
  {0x00010000, "position_control_during_sensorless"},
  {0x00020000, "vbus_under_current"},
  {0x00040000, "vbus_over_current"},
  {0x00080000, "brake_resistor_disarmed"},
  {0x00100000, "thermistor_disconnected"},
  {0x00200000, "calibration_error"},
  {0x00400000, "unknown_current_command"},
  {0x00800000, "unknown_control_mode"},
  {0x01000000, "timeout"},
  {0x02000000, "gpio_rising"},
  {0x04000000, "gpio_falling"},
  {0x08000000, "error_pin_active"},
  {0x10000000, "watchdog_triggered"},
  {0x20000000, "invalid_command"},
  {0x40000000, "brake_resistor_shorted"},
  {0x80000000, "calibration_before_move"},
}};

constexpr std::array<std::pair<std::int32_t, const char *>, 19> kMotorErrorTable{{
  {0x00000001, "phase_resistance_out_of_range"},
  {0x00000002, "phase_inductance_out_of_range"},
  {0x00000004, "adc_failed"},
  {0x00000008, "drv_fault"},
  {0x00000010, "control_deadline_missed"},
  {0x00000020, "unsupported_motor_type"},
  {0x00000040, "brake_current_out_of_range"},
  {0x00000080, "modulation_magnitude"},
  {0x00000100, "brake_deadtime_violation"},
  {0x00000200, "unexpected_timer_callback"},
  {0x00000400, "current_sense_saturation"},
  {0x00000800, "current_limit_violation"},
  {0x00001000, "brake_duty_cycle_nan"},
  {0x00002000, "dc_bus_over_regen_current"},
  {0x00004000, "dc_bus_over_current"},
  {0x00008000, "modulation_is_trivial"},
  {0x00010000, "dc_bus_over_regen_voltage"},
  {0x00020000, "dc_bus_over_voltage"},
  {0x00040000, "unbalanced_phases"},
}};

constexpr std::array<std::pair<std::int32_t, const char *>, 16> kEncoderErrorTable{{
  {0x0001, "unstable_gain"},
  {0x0002, "cpr_polepairs_mismatch"},
  {0x0004, "no_response"},
  {0x0008, "unsupported_mode"},
  {0x0010, "illegal_hall_state"},
  {0x0020, "index_not_found"},
  {0x0040, "abs_spi_timeout"},
  {0x0080, "abs_spi_comm_fail"},
  {0x0100, "abs_spi_not_ready"},
  {0x0200, "hall_not_calibrated"},
  {0x0400, "abs_spi_parity_error"},
  {0x0800, "abs_spi_framing_error"},
  {0x1000, "abs_spi_invalid_crc"},
  {0x2000, "resolution_mismatch"},
  {0x4000, "cpr_out_of_range"},
  {0x8000, "no_measure"},
}};

constexpr std::array<std::pair<std::int32_t, const char *>, 16> kControllerErrorTable{{
  {0x0001, "overspeed"},
  {0x0002, "invalid_input_mode"},
  {0x0004, "unstable_gain"},
  {0x0008, "invalid_mirror_axis"},
  {0x0010, "invalid_load_encoder"},
  {0x0020, "invalid_estimate"},
  {0x0040, "invalid_circular_range"},
  {0x0080, "spinout_detected"},
  {0x0100, "invalid_velocity_ramp"},
  {0x0200, "invalid_current_command"},
  {0x0400, "velocity_limit_violation"},
  {0x0800, "position_limit_violation"},
  {0x1000, "velocity_ramp_rate_violation"},
  {0x2000, "position_discontinuity"},
  {0x4000, "not_implemented"},
  {0x8000, "trajectory_invalid"},
}};


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

bool validate_component(
  const char * label,
  double value,
  const std::optional<double> & min_limit,
  const std::optional<double> & max_limit,
  bool enforce_limits,
  std::ostringstream & message)
{
  if (!std::isfinite(value)) {
    message << label << " command is not finite";
    return false;
  }

  if (enforce_limits) {
    if (min_limit && value < *min_limit) {
      message << label << " command " << value << " is below minimum " << *min_limit;
      return false;
    }
    if (max_limit && value > *max_limit) {
      message << label << " command " << value << " exceeds maximum " << *max_limit;
      return false;
    }
  }

  return true;
}

bool validate_joint_command(
  const JointContext & joint,
  AxisControlLevel level,
  const AxisCommandState & command_state,
  std::string & reason)
{
  std::ostringstream message;
  const auto & limits = joint.command_limits;

  auto validate_position = [&]() {
      return validate_component(
        "Position",
        command_state.command_position,
        limits.position_min,
        limits.position_max,
        limits.enforce,
        message);
    };

  auto validate_velocity = [&]() {
      return validate_component(
        "Velocity",
        command_state.command_velocity,
        limits.velocity_min,
        limits.velocity_max,
        limits.enforce,
        message);
    };

  auto validate_effort = [&]() {
      return validate_component(
        "Effort",
        command_state.command_effort,
        limits.effort_min,
        limits.effort_max,
        limits.enforce,
        message);
    };

  bool valid = true;
  switch (level) {
    case AxisControlLevel::POSITION:
      valid = validate_position() && validate_velocity() && validate_effort();
      break;
    case AxisControlLevel::VELOCITY:
      valid = validate_velocity() && validate_effort();
      break;
    case AxisControlLevel::EFFORT:
      valid = validate_effort();
      break;
    case AxisControlLevel::UNDEFINED:
      valid = true;
      break;
  }

  if (!valid) {
    reason = message.str();
  } else {
    reason.clear();
  }

  return valid;
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
    JointContext joint;
    joint.serial_number = joint_config.serial_number;
    joint.axis = joint_config.axis;
    joint.enable_watchdog = joint_config.enable_watchdog;
    joint.command_limits = joint_config.command_limits;
    joints_.emplace_back(joint);
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
    float torque_constant;
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
        static_cast<float>(hardware_config_.joints[i].watchdog_timeout));
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
      log_error_transition(
        info_.joints[i].name,
        "Axis error",
        *axis_error_value,
        joints_[i].last_axis_error,
        kAxisErrorTable);
    }
    if (const auto motor_error_value = extract_error_value(joints_[i].motor_error)) {
      log_error_transition(
        info_.joints[i].name,
        "Motor error",
        *motor_error_value,
        joints_[i].last_motor_error,
        kMotorErrorTable);
    }
    if (const auto encoder_error_value = extract_error_value(joints_[i].encoder_error)) {
      log_error_transition(
        info_.joints[i].name,
        "Encoder error",
        *encoder_error_value,
        joints_[i].last_encoder_error,
        kEncoderErrorTable);
    }
    if (const auto controller_error_value = extract_error_value(joints_[i].controller_error)) {
      log_error_transition(
        info_.joints[i].name,
        "Controller error",
        *controller_error_value,
        joints_[i].last_controller_error,
        kControllerErrorTable);
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
        joints_[i], joints_[i].control_level, command_state,
        validation_error))
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
