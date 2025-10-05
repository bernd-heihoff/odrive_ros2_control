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

namespace odrive_hardware_interface
{

int perform_axis_mode_switch(
  ODriveTransport & transport,
  std::int64_t serial_number,
  int axis,
  AxisControlLevel level,
  AxisCommandState command_state,
  std::string & failing_stage)
{
  const auto write_axis_value = [&](const char * stage, std::int16_t endpoint, auto value) -> int {
      failing_stage = stage;
      if (const int status = transport.write(
          serial_number,
          axis_endpoint(endpoint, axis),
          value);
        status != 0)
      {
        return status;
      }
      return 0;
    };

  switch (level) {
    case AxisControlLevel::UNDEFINED: {
        if (const int status = write_axis_value(
            "requesting idle state",
            odrive::AXIS__REQUESTED_STATE,
            static_cast<std::int32_t>(kAxisStateIdle));
          status != 0)
        {
          return status;
        }
        failing_stage.clear();
        return 0;
      }

    case AxisControlLevel::EFFORT: {
        command_state.command_effort = command_state.state_effort;

        if (const int status = write_axis_value(
            "setting controller to torque mode",
            odrive::AXIS__CONTROLLER__CONFIG__CONTROL_MODE,
            static_cast<std::int32_t>(level));
          status != 0)
        {
          return status;
        }

        if (const int status = write_axis_value(
            "priming torque setpoint",
            odrive::AXIS__CONTROLLER__INPUT_TORQUE,
            static_cast<float>(command_state.command_effort));
          status != 0)
        {
          return status;
        }

        if (const int status = write_axis_value(
            "entering closed loop (torque)",
            odrive::AXIS__REQUESTED_STATE,
            static_cast<std::int32_t>(kAxisStateClosedLoopControl));
          status != 0)
        {
          return status;
        }
        failing_stage.clear();
        return 0;
      }

    case AxisControlLevel::VELOCITY: {
        command_state.command_velocity = command_state.state_velocity;
        command_state.command_effort = 0.0;

        if (const int status = write_axis_value(
            "setting controller to velocity mode",
            odrive::AXIS__CONTROLLER__CONFIG__CONTROL_MODE,
            static_cast<std::int32_t>(level));
          status != 0)
        {
          return status;
        }

        if (const int status = write_axis_value(
            "priming velocity setpoint",
            odrive::AXIS__CONTROLLER__INPUT_VEL,
            static_cast<float>(radians_to_turns(command_state.command_velocity)));
          status != 0)
        {
          return status;
        }

        if (const int status = write_axis_value(
            "priming torque feed-forward",
            odrive::AXIS__CONTROLLER__INPUT_TORQUE,
            static_cast<float>(command_state.command_effort));
          status != 0)
        {
          return status;
        }

        if (const int status = write_axis_value(
            "entering closed loop (velocity)",
            odrive::AXIS__REQUESTED_STATE,
            static_cast<std::int32_t>(kAxisStateClosedLoopControl));
          status != 0)
        {
          return status;
        }
        failing_stage.clear();
        return 0;
      }

    case AxisControlLevel::POSITION: {
        command_state.command_position = command_state.state_position;
        command_state.command_velocity = 0.0;
        command_state.command_effort = 0.0;

        if (const int status = write_axis_value(
            "setting controller to position mode",
            odrive::AXIS__CONTROLLER__CONFIG__CONTROL_MODE,
            static_cast<std::int32_t>(level));
          status != 0)
        {
          return status;
        }

        if (const int status = write_axis_value(
            "priming position setpoint",
            odrive::AXIS__CONTROLLER__INPUT_POS,
            static_cast<float>(radians_to_turns(command_state.command_position)));
          status != 0)
        {
          return status;
        }

        if (const int status = write_axis_value(
            "priming velocity limit",
            odrive::AXIS__CONTROLLER__INPUT_VEL,
            static_cast<float>(radians_to_turns(command_state.command_velocity)));
          status != 0)
        {
          return status;
        }

        if (const int status = write_axis_value(
            "priming torque limit",
            odrive::AXIS__CONTROLLER__INPUT_TORQUE,
            static_cast<float>(command_state.command_effort));
          status != 0)
        {
          return status;
        }

        if (const int status = write_axis_value(
            "entering closed loop (position)",
            odrive::AXIS__REQUESTED_STATE,
            static_cast<std::int32_t>(kAxisStateClosedLoopControl));
          status != 0)
        {
          return status;
        }
        failing_stage.clear();
        return 0;
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
  const auto write_axis_value = [&](const char * stage, std::int16_t endpoint, auto value) -> int {
      failing_stage = stage;
      if (const int status = transport.write(
          serial_number,
          axis_endpoint(endpoint, axis),
          value);
        status != 0)
      {
        return status;
      }
      return 0;
    };

  const auto call_axis = [&](const char * stage, std::int16_t endpoint) -> int {
      failing_stage = stage;
      const int status = transport.call(
        serial_number,
        axis_endpoint(endpoint, axis));
      if (status != 0) {
        return status;
      }
      return 0;
    };

  switch (level) {
    case AxisControlLevel::POSITION: {
        if (const int status = write_axis_value(
            "writing position command",
            odrive::AXIS__CONTROLLER__INPUT_POS,
            static_cast<float>(radians_to_turns(command_state.command_position)));
          status != 0)
        {
          return status;
        }

        if (const int status = write_axis_value(
            "writing velocity command",
            odrive::AXIS__CONTROLLER__INPUT_VEL,
            static_cast<float>(radians_to_turns(command_state.command_velocity)));
          status != 0)
        {
          return status;
        }

        if (const int status = write_axis_value(
            "writing torque command",
            odrive::AXIS__CONTROLLER__INPUT_TORQUE,
            static_cast<float>(command_state.command_effort));
          status != 0)
        {
          return status;
        }
        failing_stage.clear();
        return 0;
      }

    case AxisControlLevel::VELOCITY: {
        if (const int status = write_axis_value(
            "writing velocity command",
            odrive::AXIS__CONTROLLER__INPUT_VEL,
            static_cast<float>(radians_to_turns(command_state.command_velocity)));
          status != 0)
        {
          return status;
        }

        if (const int status = write_axis_value(
            "writing torque command",
            odrive::AXIS__CONTROLLER__INPUT_TORQUE,
            static_cast<float>(command_state.command_effort));
          status != 0)
        {
          return status;
        }
        failing_stage.clear();
        return 0;
      }

    case AxisControlLevel::EFFORT: {
        const int status = write_axis_value(
          "writing torque command",
          odrive::AXIS__CONTROLLER__INPUT_TORQUE,
          static_cast<float>(command_state.command_effort));
        failing_stage.clear();
        return status;
      }

    case AxisControlLevel::UNDEFINED: {
        if (enable_watchdog) {
          const int status = call_axis("feeding watchdog", odrive::AXIS__WATCHDOG_FEED);
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
