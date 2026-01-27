// Copyright 2026 Wackerbot
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

#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "odrive_hardware_interface/axis_control.hpp"
#include "odrive_hardware_interface/axis_utils.hpp"
#include "odrive_hardware_interface/odrive_hardware_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "test_support/mock_transport.hpp"

namespace odrive_hardware_interface
{
namespace
{
class TestHardwareInterface : public ODriveHardwareInterface
{
public:
  CallbackReturn configure(const hardware_interface::HardwareInfo & info)
  {
    info_ = info;
    return configure_from_info(info);
  }
};

hardware_interface::HardwareInfo make_single_joint_info(
  const std::string & joint_name,
  std::int64_t serial,
  int axis,
  bool enable_watchdog,
  double watchdog_timeout_sec)
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["publish_diagnostics"] = "false";

  hardware_interface::ComponentInfo joint;
  joint.name = joint_name;
  joint.parameters["serial_number"] = "123456789ABC";
  joint.parameters["axis"] = std::to_string(axis);
  joint.parameters["watchdog_timeout"] = std::to_string(watchdog_timeout_sec);
  joint.parameters["enable_watchdog"] = enable_watchdog ? "true" : "false";
  info.joints.push_back(joint);

  // The parser uses the string value but we also keep the numeric serial for expectations.
  (void)serial;
  return info;
}

TEST(CommandClaimWatchdogTest, ClaimClearsPureWatchdogExpiry)
{
  TestHardwareInterface interface;

  const std::int64_t serial = 0x123456789ABCLL;
  constexpr int axis = 0;
  constexpr float torque_constant = 2.0F;
  constexpr double watchdog_timeout_sec = 0.10;

  MockTransport * transport = nullptr;

  interface.set_transport_factory(
    [&]() {
      auto instance = std::make_unique<MockTransport>();
      transport = instance.get();

      // Activation path expectations.
      instance->expect_read(
        serial,
        axis_endpoint(odrive::AXIS__MOTOR__CONFIG__TORQUE_CONSTANT, axis),
        torque_constant);
      instance->expect_write(
        serial,
        axis_endpoint(odrive::AXIS__CONFIG__WATCHDOG_TIMEOUT, axis),
        static_cast<float>(watchdog_timeout_sec));
      instance->expect_write(
        serial,
        axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, axis),
        static_cast<bool>(false));
      instance->expect_call(serial, axis_endpoint(odrive::AXIS__CLEAR_ERRORS, axis));

      return instance;
    });

  auto info = make_single_joint_info("wheel", serial, axis, true, watchdog_timeout_sec);
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.on_activate(rclcpp_lifecycle::State{}));
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());

  // Claim transition expectations.
  transport->expect_write(
    serial,
    axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, axis),
    static_cast<bool>(true));
  transport->expect_call(serial, axis_endpoint(odrive::AXIS__WATCHDOG_FEED, axis));

  const std::int32_t axis_error_watchdog = 0x00000800;
  transport->expect_read(serial, axis_endpoint(odrive::AXIS__ERROR, axis), axis_error_watchdog);
  transport->expect_call(serial, axis_endpoint(odrive::AXIS__CLEAR_ERRORS, axis));

  // Mode switch expectations (POSITION): primes setpoints and enters closed loop.
  transport->expect_write(
    serial,
    axis_endpoint(odrive::AXIS__CONTROLLER__CONFIG__CONTROL_MODE, axis),
    static_cast<std::int32_t>(AxisControlLevel::POSITION));
  transport->expect_write(serial, axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_POS, axis), 0.0F);
  transport->expect_write(serial, axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_VEL, axis), 0.0F);
  transport->expect_write(
    serial,
    axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_TORQUE, axis),
    0.0F);
  transport->expect_write(
    serial,
    axis_endpoint(odrive::AXIS__REQUESTED_STATE, axis),
    static_cast<std::int32_t>(kAxisStateClosedLoopControl));

  std::vector<std::string> start_interfaces{"wheel/position"};
  std::vector<std::string> stop_interfaces;

  EXPECT_EQ(
    return_type::OK, interface.prepare_command_mode_switch(
      start_interfaces,
      stop_interfaces));
  EXPECT_EQ(
    return_type::OK, interface.perform_command_mode_switch(
      start_interfaces,
      stop_interfaces));
  EXPECT_TRUE(transport->expectations_satisfied());
}

TEST(CommandClaimWatchdogTest, ClaimDoesNotClearNonWatchdogFault)
{
  TestHardwareInterface interface;

  const std::int64_t serial = 0x123456789ABCLL;
  constexpr int axis = 0;
  constexpr float torque_constant = 2.0F;
  constexpr double watchdog_timeout_sec = 0.10;

  MockTransport * transport = nullptr;

  interface.set_transport_factory(
    [&]() {
      auto instance = std::make_unique<MockTransport>();
      transport = instance.get();

      instance->expect_read(
        serial,
        axis_endpoint(odrive::AXIS__MOTOR__CONFIG__TORQUE_CONSTANT, axis),
        torque_constant);
      instance->expect_write(
        serial,
        axis_endpoint(odrive::AXIS__CONFIG__WATCHDOG_TIMEOUT, axis),
        static_cast<float>(watchdog_timeout_sec));
      instance->expect_write(
        serial,
        axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, axis),
        static_cast<bool>(false));
      instance->expect_call(serial, axis_endpoint(odrive::AXIS__CLEAR_ERRORS, axis));

      return instance;
    });

  auto info = make_single_joint_info("wheel", serial, axis, true, watchdog_timeout_sec);
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.on_activate(rclcpp_lifecycle::State{}));
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());

  // Claim transition expectations.
  transport->expect_write(
    serial,
    axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, axis),
    static_cast<bool>(true));
  transport->expect_call(serial, axis_endpoint(odrive::AXIS__WATCHDOG_FEED, axis));

  // watchdog_timer_expired + another bit -> must NOT clear.
  const std::int32_t axis_error_mixed = 0x00000801;
  transport->expect_read(serial, axis_endpoint(odrive::AXIS__ERROR, axis), axis_error_mixed);

  std::vector<std::string> start_interfaces{"wheel/position"};
  std::vector<std::string> stop_interfaces;

  EXPECT_EQ(
    return_type::OK, interface.prepare_command_mode_switch(
      start_interfaces,
      stop_interfaces));
  EXPECT_EQ(
    return_type::OK, interface.perform_command_mode_switch(
      start_interfaces,
      stop_interfaces));
  EXPECT_TRUE(transport->expectations_satisfied());
}

TEST(CommandClaimWatchdogTest, UnclaimRequestsIdleAndDisablesWatchdog)
{
  TestHardwareInterface interface;

  const std::int64_t serial = 0x123456789ABCLL;
  constexpr int axis = 0;
  constexpr float torque_constant = 2.0F;
  constexpr double watchdog_timeout_sec = 0.10;

  MockTransport * transport = nullptr;

  interface.set_transport_factory(
    [&]() {
      auto instance = std::make_unique<MockTransport>();
      transport = instance.get();

      instance->expect_read(
        serial,
        axis_endpoint(odrive::AXIS__MOTOR__CONFIG__TORQUE_CONSTANT, axis),
        torque_constant);
      instance->expect_write(
        serial,
        axis_endpoint(odrive::AXIS__CONFIG__WATCHDOG_TIMEOUT, axis),
        static_cast<float>(watchdog_timeout_sec));
      instance->expect_write(
        serial,
        axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, axis),
        static_cast<bool>(false));
      instance->expect_call(serial, axis_endpoint(odrive::AXIS__CLEAR_ERRORS, axis));

      return instance;
    });

  auto info = make_single_joint_info("wheel", serial, axis, true, watchdog_timeout_sec);
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.on_activate(rclcpp_lifecycle::State{}));
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());

  // First claim (axis error 0 -> no clear).
  transport->expect_write(
    serial,
    axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, axis),
    static_cast<bool>(true));
  transport->expect_call(serial, axis_endpoint(odrive::AXIS__WATCHDOG_FEED, axis));
  const std::int32_t axis_error_ok = 0;
  transport->expect_read(serial, axis_endpoint(odrive::AXIS__ERROR, axis), axis_error_ok);

  transport->expect_write(
    serial,
    axis_endpoint(odrive::AXIS__CONTROLLER__CONFIG__CONTROL_MODE, axis),
    static_cast<std::int32_t>(AxisControlLevel::POSITION));
  transport->expect_write(serial, axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_POS, axis), 0.0F);
  transport->expect_write(serial, axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_VEL, axis), 0.0F);
  transport->expect_write(
    serial,
    axis_endpoint(odrive::AXIS__CONTROLLER__INPUT_TORQUE, axis),
    0.0F);
  transport->expect_write(
    serial,
    axis_endpoint(odrive::AXIS__REQUESTED_STATE, axis),
    static_cast<std::int32_t>(kAxisStateClosedLoopControl));

  std::vector<std::string> start_interfaces{"wheel/position"};
  std::vector<std::string> stop_interfaces;
  EXPECT_EQ(
    return_type::OK, interface.prepare_command_mode_switch(
      start_interfaces,
      stop_interfaces));
  EXPECT_EQ(
    return_type::OK, interface.perform_command_mode_switch(
      start_interfaces,
      stop_interfaces));
  EXPECT_TRUE(transport->expectations_satisfied());

  // Now unclaim.
  constexpr std::int32_t requested_idle = kAxisStateIdle;
  transport->expect_write(
    serial, axis_endpoint(odrive::AXIS__REQUESTED_STATE, axis),
    requested_idle);
  transport->expect_write(
    serial,
    axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, axis),
    static_cast<bool>(false));

  // perform_axis_mode_switch(UNDEFINED) also requests IDLE.
  transport->expect_write(
    serial, axis_endpoint(odrive::AXIS__REQUESTED_STATE, axis),
    requested_idle);

  start_interfaces.clear();
  stop_interfaces = {"wheel/position"};
  EXPECT_EQ(
    return_type::OK, interface.prepare_command_mode_switch(
      start_interfaces,
      stop_interfaces));
  EXPECT_EQ(
    return_type::OK, interface.perform_command_mode_switch(
      start_interfaces,
      stop_interfaces));
  EXPECT_TRUE(transport->expectations_satisfied());
}

}  // namespace
}  // namespace odrive_hardware_interface
