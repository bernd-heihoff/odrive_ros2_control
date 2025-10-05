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

#include "odrive_hardware_interface/axis_control.hpp"

#include <cmath>

namespace odrive_hardware_interface
{
namespace
{
constexpr double kRadiansToTurns = 1.0 / (2.0 * M_PI);
}

int perform_axis_mode_switch(
  ODriveTransport & transport,
  std::int64_t serial_number,
  int axis,
  AxisControlLevel level,
  AxisCommandState command_state,
  std::string & failing_stage)
{
  switch (level) {
    case AxisControlLevel::UNDEFINED: {
        failing_stage = "requesting idle state";
        const std::int32_t requested_state = kAxisStateIdle;
        const int status = transport.write(
          serial_number,
          axis_endpoint(odrive::AXIS__REQUESTED_STATE, axis),
          requested_state);
        failing_stage.clear();
        return status;
      }

    case AxisControlLevel::EFFORT: {
        command_state.command_effort = command_state.state_effort;

        failing_stage = "setting controller to torque mode";
        const auto control_mode = static_cast<std::int32_t>(level);
        if (const int status = transport.write(
            serial_number,
            axis_endpoint(odrive::AXIS__CONTROLLER__CONFIG__CONTROL_MODE, axis),
            control_mode);
          status != 0)
        {
          return status;
        }

        failing_stage = "priming torque setpoint";
        const float input_torque = static_cast<float>(command_state.command_effort);
        if (const int status = transport.write(
            serial_number,
            axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_TORQUE, axis),
            input_torque);
          status != 0)
        {
          return status;
        }

        failing_stage = "entering closed loop (torque)";
        const std::int32_t requested_state = kAxisStateClosedLoopControl;
        const int status = transport.write(
          serial_number,
          axis_endpoint(odrive::AXIS__REQUESTED_STATE, axis),
          requested_state);
        failing_stage.clear();
        return status;
      }

    case AxisControlLevel::VELOCITY: {
        command_state.command_velocity = command_state.state_velocity;
        command_state.command_effort = 0.0;

        failing_stage = "setting controller to velocity mode";
        const auto control_mode = static_cast<std::int32_t>(level);
        if (const int status = transport.write(
            serial_number,
            axis_endpoint(odrive::AXIS__CONTROLLER__CONFIG__CONTROL_MODE, axis),
            control_mode);
          status != 0)
        {
          return status;
        }

        failing_stage = "priming velocity setpoint";
        const float input_vel =
          static_cast<float>(command_state.command_velocity * kRadiansToTurns);
        if (const int status = transport.write(
            serial_number,
            axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_VEL, axis),
            input_vel);
          status != 0)
        {
          return status;
        }

        failing_stage = "priming torque feed-forward";
        const float input_torque = static_cast<float>(command_state.command_effort);
        if (const int status = transport.write(
            serial_number,
            axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_TORQUE, axis),
            input_torque);
          status != 0)
        {
          return status;
        }

        failing_stage = "entering closed loop (velocity)";
        const std::int32_t requested_state = kAxisStateClosedLoopControl;
        const int status = transport.write(
          serial_number,
          axis_endpoint(odrive::AXIS__REQUESTED_STATE, axis),
          requested_state);
        failing_stage.clear();
        return status;
      }

    case AxisControlLevel::POSITION: {
        command_state.command_position = command_state.state_position;
        command_state.command_velocity = 0.0;
        command_state.command_effort = 0.0;

        failing_stage = "setting controller to position mode";
        const auto control_mode = static_cast<std::int32_t>(level);
        if (const int status = transport.write(
            serial_number,
            axis_endpoint(odrive::AXIS__CONTROLLER__CONFIG__CONTROL_MODE, axis),
            control_mode);
          status != 0)
        {
          return status;
        }

        failing_stage = "priming position setpoint";
        const float input_pos =
          static_cast<float>(command_state.command_position * kRadiansToTurns);
        if (const int status = transport.write(
            serial_number,
            axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_POS, axis),
            input_pos);
          status != 0)
        {
          return status;
        }

        failing_stage = "priming velocity limit";
        const float input_vel =
          static_cast<float>(command_state.command_velocity * kRadiansToTurns);
        if (const int status = transport.write(
            serial_number,
            axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_VEL, axis),
            input_vel);
          status != 0)
        {
          return status;
        }

        failing_stage = "priming torque limit";
        const float input_torque = static_cast<float>(command_state.command_effort);
        if (const int status = transport.write(
            serial_number,
            axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_TORQUE, axis),
            input_torque);
          status != 0)
        {
          return status;
        }

        failing_stage = "entering closed loop (position)";
        const std::int32_t requested_state = kAxisStateClosedLoopControl;
        const int status = transport.write(
          serial_number,
          axis_endpoint(odrive::AXIS__REQUESTED_STATE, axis),
          requested_state);
        failing_stage.clear();
        return status;
      }
  }

  failing_stage = "unsupported control level";
  return -1;
}

int write_axis_command(
  ODriveTransport & transport,
  std::int64_t serial_number,
  int axis,
  AxisControlLevel level,
  const AxisCommandState & command_state,
  bool enable_watchdog,
  std::string & failing_stage)
{
  switch (level) {
    case AxisControlLevel::POSITION: {
        failing_stage = "writing position command";
        const float input_pos =
          static_cast<float>(command_state.command_position * kRadiansToTurns);
        if (const int status = transport.write(
            serial_number,
            axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_POS, axis),
            input_pos);
          status != 0)
        {
          return status;
        }

        failing_stage = "writing velocity command";
        const float input_vel =
          static_cast<float>(command_state.command_velocity * kRadiansToTurns);
        if (const int status = transport.write(
            serial_number,
            axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_VEL, axis),
            input_vel);
          status != 0)
        {
          return status;
        }

        failing_stage = "writing torque command";
        const float input_torque = static_cast<float>(command_state.command_effort);
        if (const int status = transport.write(
            serial_number,
            axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_TORQUE, axis),
            input_torque);
          status != 0)
        {
          return status;
        }
        failing_stage.clear();
        return 0;
      }

    case AxisControlLevel::VELOCITY: {
        failing_stage = "writing velocity command";
        const float input_vel =
          static_cast<float>(command_state.command_velocity * kRadiansToTurns);
        if (const int status = transport.write(
            serial_number,
            axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_VEL, axis),
            input_vel);
          status != 0)
        {
          return status;
        }

        failing_stage = "writing torque command";
        const float input_torque = static_cast<float>(command_state.command_effort);
        if (const int status = transport.write(
            serial_number,
            axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_TORQUE, axis),
            input_torque);
          status != 0)
        {
          return status;
        }
        failing_stage.clear();
        return 0;
      }

    case AxisControlLevel::EFFORT: {
        failing_stage = "writing torque command";
        const float input_torque = static_cast<float>(command_state.command_effort);
        const int status = transport.write(
          serial_number,
          axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_TORQUE, axis),
          input_torque);
        failing_stage.clear();
        return status;
      }

    case AxisControlLevel::UNDEFINED: {
        if (enable_watchdog) {
          failing_stage = "feeding watchdog";
          const int status = transport.call(
            serial_number,
            axis_endpoint(odrive::AXIS__WATCHDOG_FEED, axis));
          failing_stage.clear();
          return status;
        }
        failing_stage.clear();
        return 0;
      }
  }

  failing_stage = "unsupported control level";
  return -1;
}
}  // namespace odrive_hardware_interface
