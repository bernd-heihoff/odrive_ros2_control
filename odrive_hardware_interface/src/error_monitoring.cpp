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
#include <cinttypes>
#include <optional>
#include <sstream>
#include <utility>

#include "rclcpp/rclcpp.hpp"

namespace odrive_hardware_interface
{
namespace
{
constexpr const char kLoggerName[] = "ODriveHardwareInterface";

constexpr std::array<std::pair<std::uint64_t, const char *>, 12> kAxisErrorTable{{
  {0x00000001, "invalid_state"},
  {0x00000040, "motor_failed"},
  {0x00000080, "sensorless_estimator_failed"},
  {0x00000100, "encoder_failed"},
  {0x00000200, "controller_failed"},
  {0x00000800, "watchdog_timer_expired"},
  {0x00001000, "min_endstop_pressed"},
  {0x00002000, "max_endstop_pressed"},
  {0x00004000, "estop_requested"},
  {0x00020000, "homing_without_endstop"},
  {0x00040000, "over_temp"},
  {0x00080000, "unknown_position"},
}};

constexpr std::array<std::pair<std::uint64_t, const char *>, 27> kMotorErrorTable{{
  {0x00000001, "phase_resistance_out_of_range"},
  {0x00000002, "phase_inductance_out_of_range"},
  {0x00000008, "drv_fault"},
  {0x00000010, "control_deadline_missed"},
  {0x00000080, "modulation_magnitude"},
  {0x00000400, "current_sense_saturation"},
  {0x00001000, "current_limit_violation"},
  {0x00010000, "modulation_is_nan"},
  {0x00020000, "motor_thermistor_over_temp"},
  {0x00040000, "fet_thermistor_over_temp"},
  {0x00080000, "timer_update_missed"},
  {0x00100000, "current_measurement_unavailable"},
  {0x00200000, "controller_failed"},
  {0x00400000, "i_bus_out_of_range"},
  {0x00800000, "brake_resistor_disarmed"},
  {0x01000000, "system_level"},
  {0x02000000, "bad_timing"},
  {0x04000000, "unknown_phase_estimate"},
  {0x08000000, "unknown_phase_vel"},
  {0x10000000, "unknown_torque"},
  {0x20000000, "unknown_current_command"},
  {0x40000000, "unknown_current_measurement"},
  {0x80000000, "unknown_vbus_voltage"},
  {0x100000000, "unknown_voltage_command"},
  {0x200000000, "unknown_gains"},
  {0x400000000, "controller_initializing"},
  {0x800000000, "unbalanced_phases"},
}};

constexpr std::array<std::pair<std::uint64_t, const char *>, 10> kEncoderErrorTable{{
  {0x00000001, "unstable_gain"},
  {0x00000002, "cpr_polepairs_mismatch"},
  {0x00000004, "no_response"},
  {0x00000008, "unsupported_encoder_mode"},
  {0x00000010, "illegal_hall_state"},
  {0x00000020, "index_not_found_yet"},
  {0x00000040, "abs_spi_timeout"},
  {0x00000080, "abs_spi_com_fail"},
  {0x00000100, "abs_spi_not_ready"},
  {0x00000200, "hall_not_calibrated_yet"},
}};

constexpr std::array<std::pair<std::uint64_t, const char *>, 8> kControllerErrorTable{{
  {0x00000001, "overspeed"},
  {0x00000002, "invalid_input_mode"},
  {0x00000004, "unstable_gain"},
  {0x00000008, "invalid_mirror_axis"},
  {0x00000010, "invalid_load_encoder"},
  {0x00000020, "invalid_estimate"},
  {0x00000040, "invalid_circular_range"},
  {0x00000080, "spinout_detected"},
}};

constexpr std::array<std::pair<std::uint64_t, const char *>, 8> kODriveErrorTable{{
  {0x00000001ULL, "control_iteration_missed"},
  {0x00000002ULL, "dc_bus_under_voltage"},
  {0x00000004ULL, "dc_bus_over_voltage"},
  {0x00000008ULL, "dc_bus_over_regen_current"},
  {0x00000010ULL, "dc_bus_over_current"},
  {0x00000020ULL, "brake_deadtime_violation"},
  {0x00000040ULL, "brake_duty_cycle_nan"},
  {0x00000080ULL, "invalid_brake_resistance"},
}};

template<std::size_t TableSize>
void log_error_transition(
  const std::string & joint_name,
  const char * label,
  std::uint64_t current,
  std::uint64_t & last,
  const std::array<std::pair<std::uint64_t, const char *>, TableSize> & table)
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
    "%s for joint '%s' changed from 0x%016" PRIx64 " to 0x%016" PRIx64 "%s%s%s",
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

std::optional<std::uint64_t> extract_error_value(double value)
{
  if (!std::isfinite(value)) {
    return std::nullopt;
  }

  if (value < 0.0) {
    return std::nullopt;
  }

  const auto cast_value = static_cast<std::uint64_t>(value);
  if (cast_value == 0) {
    return std::nullopt;
  }

  return cast_value;
}

void log_axis_error_transition(
  const std::string & joint_name,
  std::uint64_t current,
  std::uint64_t & last)
{
  log_error_transition(joint_name, "Axis error", current, last, kAxisErrorTable);
}

void log_motor_error_transition(
  const std::string & joint_name,
  std::uint64_t current,
  std::uint64_t & last)
{
  log_error_transition(joint_name, "Motor error", current, last, kMotorErrorTable);
}

void log_encoder_error_transition(
  const std::string & joint_name,
  std::uint64_t current,
  std::uint64_t & last)
{
  log_error_transition(joint_name, "Encoder error", current, last, kEncoderErrorTable);
}

void log_controller_error_transition(
  const std::string & joint_name,
  std::uint64_t current,
  std::uint64_t & last)
{
  log_error_transition(joint_name, "Controller error", current, last, kControllerErrorTable);
}

void log_odrive_error_transition(
  const std::string & joint_name,
  std::uint64_t current,
  std::uint64_t & last)
{
  log_error_transition(joint_name, "ODrive error", current, last, kODriveErrorTable);
}
}  // namespace odrive_hardware_interface
