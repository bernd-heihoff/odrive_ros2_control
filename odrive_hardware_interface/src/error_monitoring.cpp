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

#include "odrive_hardware_interface/error_monitoring.hpp"

#include <array>
#include <cmath>
#include <optional>
#include <sstream>
#include <utility>

#include "rclcpp/rclcpp.hpp"

namespace odrive_hardware_interface
{
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

template<std::size_t TableSize>
void log_error_transition(
  const std::string & joint_name,
  const char * label,
  std::int32_t current,
  std::int32_t & last,
  const std::array<std::pair<std::int32_t, const char *>, TableSize> & table)
{
  if (current == last) {
    return;
  }

  std::ostringstream decoded;
  bool first = true;
  for (const auto & entry : table) {
    if ((current & entry.first) != 0) {
      if (!first) {
        decoded << ", ";
      }
      decoded << entry.second;
      first = false;
    }
  }

  const auto description = decoded.str();

  RCLCPP_WARN(
    rclcpp::get_logger(kLoggerName),
    "%s for joint '%s' changed from 0x%08x to 0x%08x%s%s%s",
    label,
    joint_name.c_str(),
    last,
    current,
    description.empty() ? "" : " (",
    description.c_str(),
    description.empty() ? "" : ")");

  last = current;
}
}  // namespace

std::optional<std::int32_t> extract_error_value(double value)
{
  if (!std::isfinite(value)) {
    return std::nullopt;
  }

  const auto cast_value = static_cast<std::int32_t>(value);
  if (cast_value == 0) {
    return std::nullopt;
  }

  return cast_value;
}

void log_axis_error_transition(
  const std::string & joint_name,
  std::int32_t current,
  std::int32_t & last)
{
  log_error_transition(joint_name, "Axis error", current, last, kAxisErrorTable);
}

void log_motor_error_transition(
  const std::string & joint_name,
  std::int32_t current,
  std::int32_t & last)
{
  log_error_transition(joint_name, "Motor error", current, last, kMotorErrorTable);
}

void log_encoder_error_transition(
  const std::string & joint_name,
  std::int32_t current,
  std::int32_t & last)
{
  log_error_transition(joint_name, "Encoder error", current, last, kEncoderErrorTable);
}

void log_controller_error_transition(
  const std::string & joint_name,
  std::int32_t current,
  std::int32_t & last)
{
  log_error_transition(joint_name, "Controller error", current, last, kControllerErrorTable);
}
}  // namespace odrive_hardware_interface
