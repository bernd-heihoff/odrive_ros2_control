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

#include <cstdint>
#include <string>

#include "hardware_interface/hardware_info.hpp"
#include "odrive_hardware_interface/axis_utils.hpp"
#include "odrive_hardware_interface/odrive_hardware_interface.hpp"
#include "test_support/mock_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace odrive_hardware_interface
{
namespace
{
TEST(LifecycleTest, RecoverReinitializesTransport)
{
  ODriveHardwareInterface interface;
  const std::int64_t serial = 0x00000000000000E1LL;
  const int axis = 0;
  const float torque_constant = 6.0F;

  MockTransport * transport = nullptr;
  int creation_index = 0;

  interface.set_transport_factory(
    [&]() {
      auto instance = std::make_unique<MockTransport>();
      transport = instance.get();

      if (creation_index == 0) {
        instance->expect_read(
          serial,
          axis_endpoint(odrive::AXIS__MOTOR__CONFIG__TORQUE_CONSTANT, axis),
          torque_constant);
        instance->expect_write(
          serial,
          axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, axis),
          static_cast<bool>(false));
      } else if (creation_index == 1) {
        instance->expect_read(
          serial,
          axis_endpoint(odrive::AXIS__MOTOR__CONFIG__TORQUE_CONSTANT, axis),
          torque_constant);
        instance->expect_write(
          serial,
          axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, axis),
          static_cast<bool>(false));
        instance->expect_call(
          serial,
          axis_endpoint(odrive::AXIS__CLEAR_ERRORS, axis));
      }
      ++creation_index;
      return instance;
    });

  hardware_interface::HardwareInfo info;

  hardware_interface::ComponentInfo sensor;
  sensor.name = "bus";
  sensor.parameters["serial_number"] = "e1";
  info.sensors.push_back(sensor);

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = "e1";
  joint.parameters["axis"] = "0";
  joint.parameters["watchdog_timeout"] = "0.10";
  joint.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint);

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.on_init(info));
  ASSERT_EQ(1, creation_index);
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.on_cleanup(rclcpp_lifecycle::State{}));

  EXPECT_EQ(CallbackReturn::SUCCESS, interface.recover());
  ASSERT_EQ(2, creation_index);
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());
}
}  // namespace
}  // namespace odrive_hardware_interface
