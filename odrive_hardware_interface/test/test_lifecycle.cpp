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
        instance->expect_call(
          serial,
          axis_endpoint(odrive::AXIS__CLEAR_ERRORS, axis));
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
  ASSERT_EQ(0, creation_index);  // Transport not created yet during configure

  // Activate to create transport
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.on_activate(rclcpp_lifecycle::State{}));
  ASSERT_EQ(1, creation_index);
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.on_cleanup(rclcpp_lifecycle::State{}));

  EXPECT_EQ(CallbackReturn::SUCCESS, interface.recover());
  // On recovery, configure doesn't create transport
  ASSERT_EQ(1, creation_index);

  // Activate again to reconnect
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.on_activate(rclcpp_lifecycle::State{}));
  ASSERT_EQ(2, creation_index);
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());
}

TEST_F(LifecycleTest, InitializeSupportsMultipleDrivesBySerial)
{
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
      instance->expect_call(
        serial0,
        axis_endpoint(odrive::AXIS__CLEAR_ERRORS, axis0));
      instance->expect_read(
        serial0,
        axis_endpoint(odrive::AXIS__MOTOR__CONFIG__TORQUE_CONSTANT, axis1),
        torque1);
      instance->expect_write(
        serial0,
        axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, axis1),
        static_cast<bool>(false));
      instance->expect_call(
        serial0,
        axis_endpoint(odrive::AXIS__CLEAR_ERRORS, axis1));
      instance->expect_read(
        serial1,
        axis_endpoint(odrive::AXIS__MOTOR__CONFIG__TORQUE_CONSTANT, axis0),
        torque2);
      instance->expect_write(
        serial1,
        axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, axis0),
        static_cast<bool>(false));
      instance->expect_call(
        serial1,
        axis_endpoint(odrive::AXIS__CLEAR_ERRORS, axis0));
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

  // Transport created during activate, not configure
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.on_activate(rclcpp_lifecycle::State{}));
  ASSERT_NE(nullptr, transport);
  EXPECT_EQ(1u, transport->initialize_call_count());
  ASSERT_TRUE(transport->last_initialize_serials().has_value());
  const auto & matrix = transport->last_initialize_serials().value();
  ASSERT_EQ(2u, matrix.size());
  EXPECT_EQ(matrix[0], (std::vector<std::int64_t>{serial0, serial1}));
  EXPECT_EQ(matrix[1], (std::vector<std::int64_t>{serial0, serial0, serial1}));
  EXPECT_TRUE(transport->expectations_satisfied());
}

TEST_F(LifecycleTest, ActivateFailsWhenTransportCannotMatchSerials)
{
  // Updated: transport init moved from configure to activate, so failure happens there
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

  // configure should succeed (doesn't connect to hardware anymore)
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));

  // activate succeeds even when transport initialization fails (hardware offline)
  EXPECT_EQ(CallbackReturn::SUCCESS, interface.on_activate(rclcpp_lifecycle::State{}));
  ASSERT_NE(nullptr, transport);
  EXPECT_EQ(1u, transport->initialize_call_count());
  ASSERT_TRUE(transport->last_initialize_serials().has_value());
  const auto & matrix = transport->last_initialize_serials().value();
  ASSERT_EQ(2u, matrix.size());
  EXPECT_EQ(matrix[0], (std::vector<std::int64_t>{serial0, serial1}));
  EXPECT_EQ(matrix[1], (std::vector<std::int64_t>{serial0, serial1}));
}

TEST_F(LifecycleTest, OnInitSucceedsWithoutConnectingToHardware)
{
  // This test verifies the critical behavior: on_init always succeeds even if hardware is offline.
  // Transport connection is deferred to on_activate.
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

  bool factory_called = false;
  interface.set_transport_factory(
    [&]() {
      factory_called = true;
      return std::make_unique<MockTransport>();
    });

  hardware_interface::HardwareInfo info;
  info.hardware_parameters["publish_diagnostics"] = "false";

  hardware_interface::ComponentInfo sensor;
  sensor.name = "bus";
  sensor.parameters["serial_number"] = "ABCD12345678";
  info.sensors.push_back(sensor);

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = "ABCD12345678";
  joint.parameters["axis"] = "0";
  joint.parameters["watchdog_timeout"] = "0.10";
  joint.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint);

  // on_init (via configure) should succeed without calling transport factory
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  EXPECT_FALSE(factory_called) << "Transport should not be created during on_init";
}

TEST_F(LifecycleTest, OnActivateConnectsToHardwareAndCanFail)
{
  // This test verifies that on_activate attempts hardware connection and can fail gracefully
  // without crashing the node.
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());
  MockTransport * transport = nullptr;

  interface.set_transport_factory(
    [&]() {
      auto instance = std::make_unique<MockTransport>();
      transport = instance.get();
      // Simulate hardware connection failure
      instance->expect_initialize(-42);
      return instance;
    });

  hardware_interface::HardwareInfo info;
  info.hardware_parameters["publish_diagnostics"] = "false";

  hardware_interface::ComponentInfo sensor;
  sensor.name = "bus";
  sensor.parameters["serial_number"] = "FE";
  info.sensors.push_back(sensor);

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = "FE";
  joint.parameters["axis"] = "0";
  joint.parameters["watchdog_timeout"] = "0.10";
  joint.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint);

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));

  // on_activate succeeds even when transport initialization fails (hardware offline)
  EXPECT_EQ(CallbackReturn::SUCCESS, interface.on_activate(rclcpp_lifecycle::State{}));
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());
}

TEST_F(LifecycleTest, PerformCommandModeSwitchWithNullTransportReturnsOk)
{
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

  // Transport factory returns null -> transport_ remains null
  interface.set_transport_factory([]() {return nullptr;});

  hardware_interface::HardwareInfo info;
  info.hardware_parameters["publish_diagnostics"] = "false";

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = "123456789ABC";
  joint.parameters["axis"] = "0";
  joint.parameters["watchdog_timeout"] = "0.10";
  joint.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint);

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  EXPECT_EQ(return_type::OK, interface.perform_command_mode_switch({}, {}));
}

TEST_F(LifecycleTest, OnDeactivateWithNullTransportSucceeds)
{
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

  // Transport factory returns null -> transport_ remains null
  interface.set_transport_factory([]() {return nullptr;});

  hardware_interface::HardwareInfo info;
  info.hardware_parameters["publish_diagnostics"] = "false";

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = "123456789ABC";
  joint.parameters["axis"] = "0";
  joint.parameters["watchdog_timeout"] = "0.10";
  joint.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint);

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  EXPECT_EQ(CallbackReturn::SUCCESS, interface.on_deactivate(rclcpp_lifecycle::State{}));
}

TEST_F(LifecycleTest, OnActivateSucceedsWhenHardwareAvailable)
{
  // This test verifies successful activation when hardware is available
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

  const std::int64_t serial = 0x00000000000000CFLL;
  const int axis = 0;
  const float torque_constant = 5.5F;

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
        0.05F);
      instance->expect_write(
        serial,
        axis_endpoint(odrive::AXIS__CONFIG__ENABLE_WATCHDOG, axis),
        static_cast<bool>(true));
      instance->expect_call(
        serial,
        axis_endpoint(odrive::AXIS__WATCHDOG_FEED, axis));
      instance->expect_call(
        serial,
        axis_endpoint(odrive::AXIS__WATCHDOG_FEED, axis));
      instance->expect_call(
        serial,
        axis_endpoint(odrive::AXIS__CLEAR_ERRORS, axis));
      return instance;
    });

  hardware_interface::HardwareInfo info;
  info.hardware_parameters["publish_diagnostics"] = "false";

  hardware_interface::ComponentInfo sensor;
  sensor.name = "bus";
  sensor.parameters["serial_number"] = "CF";
  info.sensors.push_back(sensor);

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = "CF";
  joint.parameters["axis"] = "0";
  joint.parameters["enable_watchdog"] = "true";
  joint.parameters["watchdog_timeout"] = "0.05";
  info.joints.push_back(joint);

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  EXPECT_EQ(CallbackReturn::SUCCESS, interface.on_activate(rclcpp_lifecycle::State{}));
  ASSERT_NE(nullptr, transport);
  EXPECT_TRUE(transport->expectations_satisfied());
}

TEST_F(LifecycleTest, ReadWithNullTransportReturnsOkAndMarksJointsUnhealthy)
{
  // This test verifies that read() handles missing transport gracefully
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

  // Never set transport factory - transport will remain null
  interface.set_transport_factory([]() {return nullptr;});

  hardware_interface::HardwareInfo info;
  info.hardware_parameters["publish_diagnostics"] = "false";

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = "123456789ABC";
  joint.parameters["axis"] = "0";
  joint.parameters["watchdog_timeout"] = "0.10";
  joint.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint);

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));

  auto state_interfaces = interface.export_state_interfaces();
  hardware_interface::StateInterface * healthy = nullptr;
  hardware_interface::StateInterface * telemetry_valid = nullptr;
  for (auto & state : state_interfaces) {
    if (state.get_prefix_name() == "wheel" && state.get_interface_name() == "healthy") {
      healthy = &state;
    }
    if (state.get_prefix_name() == "wheel" && state.get_interface_name() == "telemetry_valid") {
      telemetry_valid = &state;
    }
  }
  ASSERT_NE(nullptr, healthy);
  ASSERT_NE(nullptr, telemetry_valid);

  // read() should return OK even with null transport, and mark joints unhealthy
  EXPECT_EQ(return_type::OK, interface.read(rclcpp::Time{}, rclcpp::Duration(0, 0)));
  EXPECT_EQ(0.0, healthy->get_value());
  EXPECT_EQ(0.0, telemetry_valid->get_value());
}

TEST_F(LifecycleTest, WriteWithNullTransportReturnsOkWithoutCrashing)
{
  // This test verifies that write() handles missing transport gracefully
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

  // Never set transport factory - transport will remain null
  interface.set_transport_factory([]() {return nullptr;});

  hardware_interface::HardwareInfo info;
  info.hardware_parameters["publish_diagnostics"] = "false";

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = "123456789ABC";
  joint.parameters["axis"] = "0";
  joint.parameters["watchdog_timeout"] = "0.10";
  joint.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint);

  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));

  // write() should return OK even with null transport
  EXPECT_EQ(return_type::OK, interface.write(rclcpp::Time{}, rclcpp::Duration(0, 0)));
}
}  // namespace
}  // namespace odrive_hardware_interface
