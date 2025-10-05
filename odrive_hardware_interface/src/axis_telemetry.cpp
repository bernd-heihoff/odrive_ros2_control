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

#include "odrive_hardware_interface/axis_telemetry.hpp"

#include <cmath>
#include <cstdint>
#include <string>

#include "odrive_hardware_interface/axis_utils.hpp"

namespace odrive_hardware_interface
{
namespace
{
constexpr double kTurnsToRadians = 2.0 * M_PI;
}

int read_axis_telemetry(
  ODriveTransport & transport,
  std::int64_t serial_number,
  int axis,
  float torque_constant,
  AxisTelemetryBuffers buffers,
  std::string & failing_stage)
{
  float iq_measured = 0.0F;
  failing_stage = "reading motor current";
  if (const int status = transport.read(
      serial_number,
      axis_endpoint(odrive::AXIS__MOTOR__CURRENT_CONTROL__IQ_MEASURED, axis),
      iq_measured);
    status != 0)
  {
    return status;
  }
  buffers.effort = static_cast<double>(iq_measured * torque_constant);

  float vel_estimate = 0.0F;
  failing_stage = "reading velocity estimate";
  if (const int status = transport.read(
      serial_number,
      axis_endpoint(odrive::AXIS__ENCODER__VEL_ESTIMATE, axis),
      vel_estimate);
    status != 0)
  {
    return status;
  }
  buffers.velocity = static_cast<double>(vel_estimate * kTurnsToRadians);

  float pos_estimate = 0.0F;
  failing_stage = "reading position estimate";
  if (const int status = transport.read(
      serial_number,
      axis_endpoint(odrive::AXIS__ENCODER__POS_ESTIMATE, axis),
      pos_estimate);
    status != 0)
  {
    return status;
  }
  buffers.position = static_cast<double>(pos_estimate * kTurnsToRadians);

  int32_t axis_error = 0;
  failing_stage = "reading axis error";
  if (const int status = transport.read(
      serial_number,
      axis_endpoint(odrive::AXIS__ERROR, axis),
      axis_error);
    status != 0)
  {
    return status;
  }
  buffers.axis_error = static_cast<double>(axis_error);

  int32_t motor_error = 0;
  failing_stage = "reading motor error";
  if (const int status = transport.read(
      serial_number,
      axis_endpoint(odrive::AXIS__MOTOR__ERROR, axis),
      motor_error);
    status != 0)
  {
    return status;
  }
  buffers.motor_error = static_cast<double>(motor_error);

  int32_t encoder_error = 0;
  failing_stage = "reading encoder error";
  if (const int status = transport.read(
      serial_number,
      axis_endpoint(odrive::AXIS__ENCODER__ERROR, axis),
      encoder_error);
    status != 0)
  {
    return status;
  }
  buffers.encoder_error = static_cast<double>(encoder_error);

  int32_t controller_error = 0;
  failing_stage = "reading controller error";
  if (const int status = transport.read(
      serial_number,
      axis_endpoint(odrive::AXIS__CONTROLLER__ERROR, axis),
      controller_error);
    status != 0)
  {
    return status;
  }
  buffers.controller_error = static_cast<double>(controller_error);

  float fet_temperature = NAN;
  failing_stage = "reading fet temperature";
  if (const int status = transport.read(
      serial_number,
      axis_endpoint(odrive::AXIS__FET_THERMISTOR__TEMPERATURE, axis),
      fet_temperature);
    status != 0)
  {
    return status;
  }
  buffers.fet_temperature = static_cast<double>(fet_temperature);

  float motor_temperature = NAN;
  failing_stage = "reading motor temperature";
  if (const int status = transport.read(
      serial_number,
      axis_endpoint(odrive::AXIS__MOTOR_THERMISTOR__TEMPERATURE, axis),
      motor_temperature);
    status != 0)
  {
    return status;
  }
  buffers.motor_temperature = static_cast<double>(motor_temperature);

  failing_stage.clear();
  return 0;
}
}  // namespace odrive_hardware_interface
