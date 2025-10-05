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

#include "odrive_hardware_interface/odrive_transport.hpp"

namespace odrive_hardware_interface
{
struct AxisTelemetryBuffers
{
  double & effort;
  double & velocity;
  double & position;
  double & axis_error;
  double & motor_error;
  double & encoder_error;
  double & controller_error;
  double & fet_temperature;
  double & motor_temperature;
};

/// Read all telemetry values for a single axis.
/// \returns 0 on success, otherwise the first transport error code encountered.
int read_axis_telemetry(
  ODriveTransport & transport,
  std::int64_t serial_number,
  int axis,
  float torque_constant,
  AxisTelemetryBuffers buffers,
  std::string & failing_stage);
}  // namespace odrive_hardware_interface
