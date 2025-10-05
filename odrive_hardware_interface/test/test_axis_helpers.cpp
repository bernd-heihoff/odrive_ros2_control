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
#include <cstring>
#include <deque>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "odrive_hardware_interface/axis_control.hpp"
#include "odrive_hardware_interface/axis_telemetry.hpp"
#include "odrive_hardware_interface/odrive_configuration.hpp"
#include "odrive_hardware_interface/odrive_transport.hpp"

namespace odrive_hardware_interface
{
namespace
{
// Minimal mock transport capturing read / write / call operations.
class MockTransport : public ODriveTransport
{
public:
  struct ReadExpectation
  {
    std::int64_t serial;
    std::int16_t endpoint;
    std::vector<std::uint8_t> payload;
    int result;
  };

  struct WriteExpectation
  {
    std::int64_t serial;
    std::int16_t endpoint;
    std::vector<std::uint8_t> payload;
    int result;
  };

  struct CallExpectation
  {
    std::int64_t serial;
    std::int16_t endpoint;
    int result;
  };

  int initialize(const SerialMatrix &) override {return 0;}

  template<typename T>
  void expect_read(std::int64_t serial, std::int16_t endpoint, const T & value, int result = 0)
  {
    ReadExpectation expectation;
    expectation.serial = serial;
    expectation.endpoint = endpoint;
    expectation.payload.resize(sizeof(T));
    std::memcpy(expectation.payload.data(), &value, sizeof(T));
    expectation.result = result;
    read_expectations_.push_back(expectation);
  }

  template<typename T>
  void expect_write(std::int64_t serial, std::int16_t endpoint, const T & value, int result = 0)
  {
    WriteExpectation expectation;
    expectation.serial = serial;
    expectation.endpoint = endpoint;
    expectation.payload.resize(sizeof(T));
    std::memcpy(expectation.payload.data(), &value, sizeof(T));
    expectation.result = result;
    write_expectations_.push_back(expectation);
  }

  void expect_call(std::int64_t serial, std::int16_t endpoint, int result = 0)
  {
    call_expectations_.push_back(CallExpectation{serial, endpoint, result});
  }

  bool expectations_satisfied() const
  {
    return read_expectations_.empty() && write_expectations_.empty() && call_expectations_.empty();
  }

  std::size_t pending_writes() const {return write_expectations_.size();}

protected:
  int read_impl(
    std::int64_t serial_number, std::int16_t endpoint_id, void * value, std::size_t size) override
  {
    if (read_expectations_.empty()) {
      ADD_FAILURE() << "Unexpected read for endpoint " << endpoint_id;
      return -1;
    }
    auto expectation = read_expectations_.front();
    read_expectations_.pop_front();
    EXPECT_EQ(expectation.serial, serial_number);
    EXPECT_EQ(expectation.endpoint, endpoint_id);
    if (expectation.result != 0) {
      return expectation.result;
    }
    EXPECT_EQ(expectation.payload.size(), size);
    std::memcpy(value, expectation.payload.data(), size);
    return 0;
  }

  int write_impl(
    std::int64_t serial_number, std::int16_t endpoint_id, const void * value,
    std::size_t size) override
  {
    if (write_expectations_.empty()) {
      ADD_FAILURE() << "Unexpected write for endpoint " << endpoint_id;
      return -1;
    }
    auto expectation = write_expectations_.front();
    write_expectations_.pop_front();
    EXPECT_EQ(expectation.serial, serial_number);
    EXPECT_EQ(expectation.endpoint, endpoint_id);
    if (expectation.payload.size() != size) {
      ADD_FAILURE() << "Write size mismatch";
    } else {
      std::vector<std::uint8_t> actual(size);
      std::memcpy(actual.data(), value, size);
      EXPECT_EQ(expectation.payload, actual);
    }
    return expectation.result;
  }

  int call(std::int64_t serial_number, std::int16_t endpoint_id) override
  {
    if (call_expectations_.empty()) {
      ADD_FAILURE() << "Unexpected call for endpoint " << endpoint_id;
      return -1;
    }
    auto expectation = call_expectations_.front();
    call_expectations_.pop_front();
    EXPECT_EQ(expectation.serial, serial_number);
    EXPECT_EQ(expectation.endpoint, endpoint_id);
    return expectation.result;
  }

private:
  std::deque<ReadExpectation> read_expectations_;
  std::deque<WriteExpectation> write_expectations_;
  std::deque<CallExpectation> call_expectations_;
};

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

  // Mimic controllers populating command buffers prior to write.
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

TEST(AxisControlTest, WriteCommandFeedsWatchdogWhenUndefined)
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

  const auto watchdog_endpoint = axis_endpoint(odrive::AXIS__WATCHDOG_FEED, kAxis);
  transport.expect_call(kSerial, watchdog_endpoint);

  std::string failing_stage;
  const int status = write_axis_command(
    transport, kSerial, kAxis, AxisControlLevel::UNDEFINED, command_state, true, failing_stage);

  EXPECT_EQ(0, status);
  EXPECT_TRUE(failing_stage.empty());
  EXPECT_TRUE(transport.expectations_satisfied());
}

TEST(AxisTelemetryTest, ReadTelemetryAppliesConversions)
{
  MockTransport transport;

  double telemetry_effort = std::numeric_limits<double>::quiet_NaN();
  double telemetry_velocity = std::numeric_limits<double>::quiet_NaN();
  double telemetry_position = std::numeric_limits<double>::quiet_NaN();
  double telemetry_axis_error = std::numeric_limits<double>::quiet_NaN();
  double telemetry_motor_error = std::numeric_limits<double>::quiet_NaN();
  double telemetry_encoder_error = std::numeric_limits<double>::quiet_NaN();
  double telemetry_controller_error = std::numeric_limits<double>::quiet_NaN();
  double telemetry_fet_temperature = std::numeric_limits<double>::quiet_NaN();
  double telemetry_motor_temperature = std::numeric_limits<double>::quiet_NaN();

  AxisTelemetryBuffers buffers{
    telemetry_effort,
    telemetry_velocity,
    telemetry_position,
    telemetry_axis_error,
    telemetry_motor_error,
    telemetry_encoder_error,
    telemetry_controller_error,
    telemetry_fet_temperature,
    telemetry_motor_temperature};

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
    transport, kSerial, kAxis, torque_constant, buffers, failing_stage);

  EXPECT_EQ(0, status);
  EXPECT_TRUE(failing_stage.empty());

  EXPECT_DOUBLE_EQ(iq_measured * torque_constant, telemetry_effort);
  EXPECT_NEAR(vel_estimate * 2.0 * M_PI, telemetry_velocity, 1e-6);
  EXPECT_NEAR(pos_estimate * 2.0 * M_PI, telemetry_position, 1e-6);
  EXPECT_DOUBLE_EQ(axis_error, telemetry_axis_error);
  EXPECT_DOUBLE_EQ(motor_error, telemetry_motor_error);
  EXPECT_DOUBLE_EQ(encoder_error, telemetry_encoder_error);
  EXPECT_DOUBLE_EQ(controller_error, telemetry_controller_error);
  EXPECT_DOUBLE_EQ(fet_temp, telemetry_fet_temperature);
  EXPECT_DOUBLE_EQ(motor_temp, telemetry_motor_temperature);

  EXPECT_TRUE(transport.expectations_satisfied());
}

TEST(AxisTelemetryTest, PropagatesTransportErrorStage)
{
  MockTransport transport;
  double effort = 0.0;
  double velocity = 0.0;
  double position = 0.0;
  double axis_error = 0.0;
  double motor_error = 0.0;
  double encoder_error = 0.0;
  double controller_error = 0.0;
  double fet_temp = 0.0;
  double motor_temp = 0.0;

  AxisTelemetryBuffers buffers{
    effort,
    velocity,
    position,
    axis_error,
    motor_error,
    encoder_error,
    controller_error,
    fet_temp,
    motor_temp};

  transport.expect_read(
    kSerial, axis_endpoint(odrive::AXIS__MOTOR__CURRENT_CONTROL__IQ_MEASURED, kAxis), 0.0F, -7);

  std::string failing_stage;
  const int status = read_axis_telemetry(
    transport, kSerial, kAxis, 1.0F, buffers, failing_stage);

  EXPECT_EQ(-7, status);
  EXPECT_EQ("reading motor current", failing_stage);
}

TEST(ConfigurationTest, ParsesValidHardwareInfo)
{
  hardware_interface::HardwareInfo info;

  hardware_interface::ComponentInfo sensor;
  sensor.name = "vbus";
  sensor.parameters["serial_number"] = "1234abcd";
  info.sensors.push_back(sensor);

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = "0001ffff";
  joint.parameters["axis"] = "1";
  joint.parameters["watchdog_timeout"] = "0.25";
  joint.parameters["enable_watchdog"] = "true";
  info.joints.push_back(joint);

  HardwareConfiguration config;
  std::string error;
  ASSERT_TRUE(parse_hardware_configuration(info, config, error)) << error;

  ASSERT_EQ(1u, config.sensors.size());
  EXPECT_EQ("vbus", config.sensors.front().name);
  EXPECT_EQ(0x1234abcd, config.sensors.front().serial_number);

  ASSERT_EQ(1u, config.joints.size());
  const auto & parsed_joint = config.joints.front();
  EXPECT_EQ("wheel", parsed_joint.name);
  EXPECT_EQ(0x0001ffff, parsed_joint.serial_number);
  EXPECT_EQ(1, parsed_joint.axis);
  EXPECT_TRUE(parsed_joint.enable_watchdog);
  EXPECT_DOUBLE_EQ(0.25, parsed_joint.watchdog_timeout);
}

TEST(ConfigurationTest, ReportsMissingParameters)
{
  hardware_interface::HardwareInfo info;

  hardware_interface::ComponentInfo sensor;
  sensor.name = "vbus";
  sensor.parameters["serial_number"] = "1234";
  info.sensors.push_back(sensor);

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = "0001";
  // Missing axis parameter
  joint.parameters["watchdog_timeout"] = "0.1";
  joint.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint);

  HardwareConfiguration config;
  std::string error;
  EXPECT_FALSE(parse_hardware_configuration(info, config, error));
  EXPECT_NE(std::string::npos, error.find("axis"));
}

}  // namespace
}  // namespace odrive_hardware_interface
