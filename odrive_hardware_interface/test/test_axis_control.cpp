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

#include "odrive_hardware_interface/axis_control.hpp"
#include "odrive_hardware_interface/axis_utils.hpp"
#include "test_support/mock_transport.hpp"

namespace odrive_hardware_interface
{
namespace
{
constexpr std::int64_t kSerial = 0x1234ABCD;
constexpr int kAxis = 1;

TEST(AxisControlTest, PositionModeSwitchWritesInitialSetpoints)
{
  MockTransport transport;
  double commanded_position = 0.0;
  double commanded_velocity = 0.0;
  double commanded_effort = 0.0;
  double state_position = 1.5;
  double state_velocity = -2.0;
  double state_effort = 0.25;

  AxisCommandState command_state{
    commanded_position,
    commanded_velocity,
    commanded_effort,
    state_position,
    state_velocity,
    state_effort};

  const auto control_mode_endpoint = axis_endpoint(
    odrive::AXIS__CONTROLLER__CONFIG__CONTROL_MODE, kAxis);
  const auto pos_endpoint = axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_POS, kAxis);
  const auto vel_endpoint = axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_VEL, kAxis);
  const auto torque_endpoint = axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_TORQUE, kAxis);
  const auto state_endpoint = axis_endpoint(odrive::AXIS__REQUESTED_STATE, kAxis);

  const std::int32_t control_mode_value = static_cast<std::int32_t>(AxisControlLevel::POSITION);
  const float expected_pos = static_cast<float>(state_position / (2.0 * M_PI));
  const float expected_vel = 0.0F;
  const float expected_torque = 0.0F;
  const std::int32_t requested_state = kAxisStateClosedLoopControl;

  transport.expect_write(kSerial, control_mode_endpoint, control_mode_value);
  transport.expect_write(kSerial, pos_endpoint, expected_pos);
  transport.expect_write(kSerial, vel_endpoint, expected_vel);
  transport.expect_write(kSerial, torque_endpoint, expected_torque);
  transport.expect_write(kSerial, state_endpoint, requested_state);

  std::string failing_stage;
  const int status = perform_axis_mode_switch(
    transport, kSerial, kAxis, AxisControlLevel::POSITION, command_state, failing_stage);

  EXPECT_EQ(0, status);
  EXPECT_TRUE(failing_stage.empty());
  EXPECT_DOUBLE_EQ(state_position, commanded_position);
  EXPECT_DOUBLE_EQ(0.0, commanded_velocity);
  EXPECT_DOUBLE_EQ(0.0, commanded_effort);
  EXPECT_TRUE(transport.expectations_satisfied());
}

TEST(AxisControlTest, ModeSwitchReturnsErrorOnTransportFailure)
{
  MockTransport transport;
  double command_pos = 0.0;
  double command_vel = 0.0;
  double command_eff = 0.0;
  double state_pos = 0.0;
  double state_vel = 0.0;
  double state_eff = 1.0;

  AxisCommandState command_state{
    command_pos,
    command_vel,
    command_eff,
    state_pos,
    state_vel,
    state_eff};

  const auto control_mode_endpoint = axis_endpoint(
    odrive::AXIS__CONTROLLER__CONFIG__CONTROL_MODE, kAxis);

  const std::int32_t control_mode_value = static_cast<std::int32_t>(AxisControlLevel::EFFORT);
  transport.expect_write(kSerial, control_mode_endpoint, control_mode_value, -42);

  std::string failing_stage;
  const int status = perform_axis_mode_switch(
    transport, kSerial, kAxis, AxisControlLevel::EFFORT, command_state, failing_stage);

  EXPECT_EQ(-42, status);
  EXPECT_EQ("setting controller to torque mode", failing_stage);
  EXPECT_EQ(0U, transport.pending_writes());
}

TEST(AxisControlTest, WriteVelocityCommandConvertsUnits)
{
  MockTransport transport;
  double command_pos = std::numeric_limits<double>::quiet_NaN();
  double command_vel = 0.0;
  double command_eff = 0.0;
  double state_pos = 0.0;
  const double desired_vel = 12.0;  // rad/s
  const double desired_eff = 0.5;
  double state_vel = desired_vel;
  double state_eff = desired_eff;

  AxisCommandState command_state{
    command_pos,
    command_vel,
    command_eff,
    state_pos,
    state_vel,
    state_eff};

  command_state.command_velocity = desired_vel;
  command_state.command_effort = desired_eff;
  EXPECT_DOUBLE_EQ(desired_vel, command_state.command_velocity);
  EXPECT_DOUBLE_EQ(desired_eff, command_state.command_effort);

  const auto vel_endpoint = axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_VEL, kAxis);
  const auto torque_endpoint = axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_TORQUE, kAxis);

  const float expected_vel = static_cast<float>(desired_vel / (2.0 * M_PI));
  const float expected_torque = static_cast<float>(desired_eff);

  transport.expect_write(kSerial, vel_endpoint, expected_vel);
  transport.expect_write(kSerial, torque_endpoint, expected_torque);

  std::string failing_stage;
  const int status = write_axis_command(
    transport, kSerial, kAxis, AxisControlLevel::VELOCITY, command_state, false, failing_stage);

  EXPECT_EQ(0, status);
  EXPECT_TRUE(failing_stage.empty());
  EXPECT_DOUBLE_EQ(state_vel, command_vel);
  EXPECT_DOUBLE_EQ(state_eff, command_eff);
  EXPECT_TRUE(transport.expectations_satisfied());
}

TEST(AxisControlTest, WriteCommandDoesNotFeedWatchdogWhenUndefined)
{
  MockTransport transport;
  double command_pos = 0.0;
  double command_vel = 0.0;
  double command_eff = 0.0;
  double state_pos = 0.0;
  double state_vel = 0.0;
  double state_eff = 0.0;

  AxisCommandState command_state{
    command_pos,
    command_vel,
    command_eff,
    state_pos,
    state_vel,
    state_eff};

  std::string failing_stage;
  const int status = write_axis_command(
    transport, kSerial, kAxis, AxisControlLevel::UNDEFINED, command_state, true, failing_stage);

  EXPECT_EQ(0, status);
  EXPECT_TRUE(failing_stage.empty());
  EXPECT_TRUE(transport.expectations_satisfied());
}

TEST(AxisControlTest, WriteCommandSkipsWatchdogWhenDisabled)
{
  MockTransport transport;
  double command_pos = 0.0;
  double command_vel = 0.0;
  double command_eff = 0.0;
  double state_pos = 0.0;
  double state_vel = 0.0;
  double state_eff = 0.0;

  AxisCommandState command_state{
    command_pos,
    command_vel,
    command_eff,
    state_pos,
    state_vel,
    state_eff};

  std::string failing_stage;
  const int status = write_axis_command(
    transport, kSerial, kAxis, AxisControlLevel::UNDEFINED, command_state, false, failing_stage);

  EXPECT_EQ(0, status);
  EXPECT_TRUE(failing_stage.empty());
  EXPECT_TRUE(transport.expectations_satisfied());
}

TEST(AxisControlTest, WritePositionCommandReportsFailingStage)
{
  MockTransport transport;
  double command_pos = 0.0;
  double command_vel = 0.0;
  double command_eff = 0.0;
  double state_pos = 0.0;
  double state_vel = 0.0;
  double state_eff = 0.0;

  AxisCommandState command_state{
    command_pos,
    command_vel,
    command_eff,
    state_pos,
    state_vel,
    state_eff};

  const auto pos_endpoint = axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_POS, kAxis);
  const auto vel_endpoint = axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_VEL, kAxis);

  transport.expect_write(kSerial, pos_endpoint, 0.0F);
  transport.expect_write(kSerial, vel_endpoint, 0.0F, -13);

  std::string failing_stage;
  const int status = write_axis_command(
    transport, kSerial, kAxis, AxisControlLevel::POSITION, command_state, false, failing_stage);

  EXPECT_EQ(-13, status);
  EXPECT_EQ("writing velocity command", failing_stage);
}
}  // namespace
}  // namespace odrive_hardware_interface
