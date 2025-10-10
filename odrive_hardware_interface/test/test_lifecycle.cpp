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
#include <cstdlib>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "odrive_hardware_interface/axis_utils.hpp"
#include "odrive_hardware_interface/odrive_hardware_interface.hpp"
#include "test_support/mock_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_support/fake_diagnostics.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace odrive_hardware_interface
{
namespace
{
class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    if (rclcpp::ok()) {
      return;
    }
    static const char * argv[] = {"odrive_lifecycle"};
    int argc = 1;
    rclcpp::init(argc, argv);
  }
};

[[maybe_unused]] ::testing::Environment * const kRclcppEnvironment =
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment());

void ensure_rclcpp_context()
{
  static std::once_flag registered_shutdown;
  if (!rclcpp::ok()) {
    static const char * argv[] = {"odrive_lifecycle"};
    int argc = 1;
    rclcpp::init(argc, argv);
  }
  std::call_once(
    registered_shutdown, []() {
      std::atexit(
        []() {
          if (rclcpp::ok()) {
            rclcpp::shutdown();
          }
        });
    });
}

class LifecycleTest : public ::testing::Test
{
};

class TestHardwareInterface : public ODriveHardwareInterface
{
public:
  CallbackReturn configure(const hardware_interface::HardwareInfo & info)
  {
    info_ = info;
    return configure_from_info(info);
  }
};

TEST_F(LifecycleTest, RecoverReinitializesTransport)
{
  ensure_rclcpp_context();
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());
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
  info.hardware_parameters["publish_diagnostics"] = "false";

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

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  ASSERT_EQ(1, creation_index);
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.on_cleanup(rclcpp_lifecycle::State{}));

  EXPECT_EQ(CallbackReturn::SUCCESS, interface.recover());
  ASSERT_EQ(2, creation_index);
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());
}

TEST_F(LifecycleTest, InitializeSupportsMultipleDrivesBySerial)
{
  ensure_rclcpp_context();
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());
  const std::int64_t serial0 = 0x0000000000000101LL;
  const std::int64_t serial1 = 0x0000000000000102LL;
  const int axis0 = 0;
  const int axis1 = 1;
  const float torque0 = 6.5F;
  const float torque1 = 7.0F;
  const float torque2 = 8.0F;

  MockTransport * transport = nullptr;
  interface.set_transport_factory(
    [&]() {
      auto instance = std::make_unique<MockTransport>();
      transport = instance.get();
      instance->expect_read(
        serial0,
        axis_endpoint(odrive::AXIS__MOTOR__CONFIG__TORQUE_CONSTANT, axis0),
        torque0);
      instance->expect_write(
        serial0,
        axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, axis0),
        static_cast<bool>(false));
      instance->expect_read(
        serial0,
        axis_endpoint(odrive::AXIS__MOTOR__CONFIG__TORQUE_CONSTANT, axis1),
        torque1);
      instance->expect_write(
        serial0,
        axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, axis1),
        static_cast<bool>(false));
      instance->expect_read(
        serial1,
        axis_endpoint(odrive::AXIS__MOTOR__CONFIG__TORQUE_CONSTANT, axis0),
        torque2);
      instance->expect_write(
        serial1,
        axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, axis0),
        static_cast<bool>(false));
      return instance;
    });

  hardware_interface::HardwareInfo info;
  info.hardware_parameters["publish_diagnostics"] = "false";

  hardware_interface::ComponentInfo sensor0;
  sensor0.name = "bus0";
  sensor0.parameters["serial_number"] = "101";
  info.sensors.push_back(sensor0);

  hardware_interface::ComponentInfo sensor1;
  sensor1.name = "bus1";
  sensor1.parameters["serial_number"] = "102";
  info.sensors.push_back(sensor1);

  hardware_interface::ComponentInfo joint0;
  joint0.name = "front_left";
  joint0.parameters["serial_number"] = "101";
  joint0.parameters["axis"] = "0";
  joint0.parameters["watchdog_timeout"] = "0.10";
  joint0.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint0);

  hardware_interface::ComponentInfo joint1;
  joint1.name = "front_right";
  joint1.parameters["serial_number"] = "101";
  joint1.parameters["axis"] = "1";
  joint1.parameters["watchdog_timeout"] = "0.10";
  joint1.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint1);

  hardware_interface::ComponentInfo joint2;
  joint2.name = "rear_left";
  joint2.parameters["serial_number"] = "102";
  joint2.parameters["axis"] = "0";
  joint2.parameters["watchdog_timeout"] = "0.10";
  joint2.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint2);

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  ASSERT_NE(nullptr, transport);
  EXPECT_EQ(1u, transport->initialize_call_count());
  ASSERT_TRUE(transport->last_initialize_serials().has_value());
  const auto & matrix = transport->last_initialize_serials().value();
  ASSERT_EQ(2u, matrix.size());
  EXPECT_EQ(matrix[0], (std::vector<std::int64_t>{serial0, serial1}));
  EXPECT_EQ(matrix[1], (std::vector<std::int64_t>{serial0, serial0, serial1}));
  EXPECT_TRUE(transport->expectations_satisfied());
}

TEST_F(LifecycleTest, InitializeFailsWhenTransportCannotMatchSerials)
{
  ensure_rclcpp_context();
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());
  const std::int64_t serial0 = 0x0000000000000201LL;
  const std::int64_t serial1 = 0x0000000000000202LL;

  MockTransport * transport = nullptr;
  interface.set_transport_factory(
    [&]() {
      auto instance = std::make_unique<MockTransport>();
      transport = instance.get();
      instance->expect_initialize(-1);
      return instance;
    });

  hardware_interface::HardwareInfo info;
  info.hardware_parameters["publish_diagnostics"] = "false";

  hardware_interface::ComponentInfo sensor0;
  sensor0.name = "bus0";
  sensor0.parameters["serial_number"] = "201";
  info.sensors.push_back(sensor0);

  hardware_interface::ComponentInfo sensor1;
  sensor1.name = "bus1";
  sensor1.parameters["serial_number"] = "202";
  info.sensors.push_back(sensor1);

  hardware_interface::ComponentInfo joint0;
  joint0.name = "wheel0";
  joint0.parameters["serial_number"] = "201";
  joint0.parameters["axis"] = "0";
  joint0.parameters["watchdog_timeout"] = "0.10";
  joint0.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint0);

  hardware_interface::ComponentInfo joint1;
  joint1.name = "wheel1";
  joint1.parameters["serial_number"] = "202";
  joint1.parameters["axis"] = "1";
  joint1.parameters["watchdog_timeout"] = "0.10";
  joint1.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint1);

  ASSERT_EQ(CallbackReturn::ERROR, interface.configure(info));
  ASSERT_NE(nullptr, transport);
  EXPECT_EQ(1u, transport->initialize_call_count());
  ASSERT_TRUE(transport->last_initialize_serials().has_value());
  const auto & matrix = transport->last_initialize_serials().value();
  ASSERT_EQ(2u, matrix.size());
  EXPECT_EQ(matrix[0], (std::vector<std::int64_t>{serial0, serial1}));
  EXPECT_EQ(matrix[1], (std::vector<std::int64_t>{serial0, serial1}));
}
}  // namespace
}  // namespace odrive_hardware_interface
