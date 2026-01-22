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

#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "odrive_hardware_interface/axis_telemetry.hpp"
#include "odrive_hardware_interface/axis_utils.hpp"
#include "test_support/mock_transport.hpp"

namespace odrive_hardware_interface
{
namespace
{
constexpr std::int64_t kSerial = 0xABCD1234;
constexpr int kAxis = 2;

TEST(AxisTelemetryTest, ReadTelemetryAppliesConversions)
{
  MockTransport transport;

  AxisTelemetrySample sample;

  const float iq_measured = 3.0F;
  const float vel_estimate = 2.0F;
  const float pos_estimate = 1.0F;
  const std::int32_t axis_error = 12;
  const std::int32_t motor_error = 34;
  const std::int32_t encoder_error = 56;
  const std::int32_t controller_error = 78;
  const float fet_temp = 47.5F;
  const float motor_temp = 51.25F;

  transport.expect_read(
    kSerial, axis_endpoint(odrive::AXIS__MOTOR__CURRENT_CONTROL__IQ_MEASURED, kAxis), iq_measured);
  transport.expect_read(
    kSerial, axis_endpoint(odrive::AXIS__ENCODER__VEL_ESTIMATE, kAxis), vel_estimate);
  transport.expect_read(
    kSerial, axis_endpoint(odrive::AXIS__ENCODER__POS_ESTIMATE, kAxis), pos_estimate);
  transport.expect_read(kSerial, axis_endpoint(odrive::AXIS__ERROR, kAxis), axis_error);
  transport.expect_read(
    kSerial, axis_endpoint(odrive::AXIS__MOTOR__ERROR, kAxis), motor_error);
  transport.expect_read(
    kSerial, axis_endpoint(odrive::AXIS__ENCODER__ERROR, kAxis), encoder_error);
  transport.expect_read(
    kSerial, axis_endpoint(odrive::AXIS__CONTROLLER__ERROR, kAxis), controller_error);
  transport.expect_read(
    kSerial, axis_endpoint(odrive::AXIS__FET_THERMISTOR__TEMPERATURE, kAxis), fet_temp);
  transport.expect_read(
    kSerial, axis_endpoint(odrive::AXIS__MOTOR_THERMISTOR__TEMPERATURE, kAxis), motor_temp);

  std::string failing_stage;
  const float torque_constant = 2.0F;
  const int status = read_axis_telemetry(
    transport, kSerial, kAxis, torque_constant, sample, failing_stage);

  EXPECT_EQ(0, status);
  EXPECT_TRUE(failing_stage.empty());

  EXPECT_DOUBLE_EQ(iq_measured * torque_constant, sample.effort);
  EXPECT_NEAR(vel_estimate * 2.0 * M_PI, sample.velocity, 1e-6);
  EXPECT_NEAR(pos_estimate * 2.0 * M_PI, sample.position, 1e-6);
  EXPECT_DOUBLE_EQ(axis_error, sample.axis_error);
  EXPECT_DOUBLE_EQ(motor_error, sample.motor_error);
  EXPECT_DOUBLE_EQ(encoder_error, sample.encoder_error);
  EXPECT_DOUBLE_EQ(controller_error, sample.controller_error);
  EXPECT_DOUBLE_EQ(fet_temp, sample.fet_temperature);
  EXPECT_DOUBLE_EQ(motor_temp, sample.motor_temperature);

  EXPECT_TRUE(transport.expectations_satisfied());
}

TEST(AxisTelemetryTest, PropagatesTransportErrorStage)
{
  MockTransport transport;

  AxisTelemetrySample sample;

  transport.expect_read(
    kSerial, axis_endpoint(odrive::AXIS__MOTOR__CURRENT_CONTROL__IQ_MEASURED, kAxis), 0.0F, -7);

  std::string failing_stage;
  const int status = read_axis_telemetry(
    transport, kSerial, kAxis, 1.0F, sample, failing_stage);

  EXPECT_EQ(-7, status);
  EXPECT_EQ("reading motor current", failing_stage);
}
}  // namespace
}  // namespace odrive_hardware_interface
