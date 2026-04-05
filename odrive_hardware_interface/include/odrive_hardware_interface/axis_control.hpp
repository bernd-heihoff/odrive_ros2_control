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

#include <cstdint>
#include <string>

#include "odrive_hardware_interface/axis_utils.hpp"
#include "odrive_hardware_interface/odrive_transport.hpp"

namespace odrive_hardware_interface
{
constexpr std::int32_t kAxisStateIdle = 1;
constexpr std::int32_t kAxisStateClosedLoopControl = 8;

/// ODrive axis control mode.
///
/// Maps to ODrive firmware's ControlMode enum. The integer values MUST match
/// the firmware's enum for proper protocol communication.
enum class AxisControlLevel : std::int32_t
{
  /// No active control (motor idle)
  UNDEFINED = 0,
  /// Torque/current control mode
  EFFORT = 1,
  /// Velocity control mode (with optional torque feed-forward)
  VELOCITY = 2,
  /// Position control mode (with optional velocity and torque limits)
  POSITION = 3
};

/// Axis command and state bundle.
///
/// Groups command references (mutable) with state feedback (const) to prevent
/// accidental modification of feedback values during command processing.
struct AxisCommandState
{
  double & command_position;  ///< Commanded position [rad]
  double & command_velocity;  ///< Commanded velocity [rad/s]
  double & command_effort;    ///< Commanded effort [Nm]
  const double & state_position;   ///< Current measured position [rad]
  const double & state_velocity;   ///< Current measured velocity [rad/s]
  const double & state_effort;     ///< Current measured effort [Nm]
};

/// Configure the controller for a new command mode and prime initial setpoints.
///
/// This function performs a multi-step sequence:
/// 1. Sets the controller mode (position/velocity/torque)
/// 2. Writes initial setpoint values based on current state
/// 3. Transitions axis to CLOSED_LOOP_CONTROL state (or IDLE for UNDEFINED)
///
/// The initial setpoints are primed to minimize sudden jumps:
/// - Position mode: primes position to current position, velocity/effort to 0
/// - Velocity mode: primes velocity to current velocity, effort to 0
/// - Effort mode: primes effort to current effort
///
/// SAFETY: Ensure axis is calibrated and encoder is ready before calling.
/// An uncalibrated axis will report errors when entering closed-loop control.
///
/// @param transport USB communication interface
/// @param serial_number ODrive serial number (decimal format, e.g., 0x000000A1)
/// @param axis Axis index (0 or 1 for dual-axis ODrive models)
/// @param level Desired control level (POSITION/VELOCITY/EFFORT/UNDEFINED)
/// @param command_state Command and state bundle with initial values
/// @param[out] failing_stage Description of which step failed (empty on success)
/// @return 0 on success, transport error code on failure
int perform_axis_mode_switch(
  ODriveTransport & transport,
  std::int64_t serial_number,
  int axis,
  AxisControlLevel level,
  AxisCommandState command_state,
  std::string & failing_stage);

/// Write the latest command setpoints for the current mode.
///
/// Transmits the appropriate command values based on the active control level:
/// - POSITION: writes position, velocity (limit), and torque (limit)
/// - VELOCITY: writes velocity and torque (feed-forward)
/// - EFFORT: writes torque only
/// - UNDEFINED: no-op (does not write setpoints and does not feed the watchdog)
///
/// WATCHDOG SAFETY DESIGN:
/// Watchdog feeds occur ONLY during successful command writes. This couples
/// watchdog health to active control - if the controller stops commanding
/// (lifecycle gating, fault masking, etc.), the watchdog expires and halts
/// the motor. This is safer than feeding based on connection health alone,
/// which could allow runaway motors if the control loop hangs.
///
/// Set watchdog_timeout conservatively above your worst-case control loop period.
///
/// @param transport USB communication interface
/// @param serial_number ODrive serial number (decimal format)
/// @param axis Axis index (0 or 1)
/// @param level Active control level (determines which commands are sent)
/// @param command_state Command values to transmit
/// @param enable_watchdog If true, feeds watchdog after successful write (ignored for UNDEFINED)
/// @param[out] failing_stage Description of which step failed (empty on success)
/// @return 0 on success, transport error code on failure
int write_axis_command(
  ODriveTransport & transport,
  std::int64_t serial_number,
  int axis,
  AxisControlLevel level,
  const AxisCommandState & command_state,
  bool enable_watchdog,
  std::string & failing_stage);
}  // namespace odrive_hardware_interface
