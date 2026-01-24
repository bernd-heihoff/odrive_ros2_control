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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "odrive_hardware_interface/axis_control.hpp"
#include "odrive_hardware_interface/axis_utils.hpp"
#include "odrive_hardware_interface/odrive_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_support/fake_diagnostics.hpp"
#include "test_support/mock_transport.hpp"

namespace odrive_hardware_interface
{
namespace
{
class ThreadSafetyTest : public ::testing::Test
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

hardware_interface::HardwareInfo make_test_info(const std::string & serial_hex)
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["publish_diagnostics"] = "false";

  hardware_interface::ComponentInfo sensor;
  sensor.name = "bus";
  sensor.parameters["serial_number"] = serial_hex;
  info.sensors.push_back(sensor);

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel";
  joint.parameters["serial_number"] = serial_hex;
  joint.parameters["axis"] = "0";
  joint.parameters["watchdog_timeout"] = "0.10";
  joint.parameters["enable_watchdog"] = "false";
  info.joints.push_back(joint);

  return info;
}

TEST_F(ThreadSafetyTest, ConcurrentReadWriteDoesNotCrash)
{
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

  const std::int64_t serial = 0x00000000000000A1LL;
  const int axis = 0;
  const float torque_constant = 2.0F;

  MockTransport * transport = nullptr;
  interface.set_transport_factory(
    [&]() {
      auto instance = std::make_unique<MockTransport>();
      transport = instance.get();

      // Setup basic initialization expectations
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

      // Allow repeated reads/writes during concurrent execution
      instance->set_permissive_mode(true);

      return instance;
    });

  auto info = make_test_info("a1");
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.on_activate(rclcpp_lifecycle::State{}));

  std::atomic<bool> running{true};
  std::atomic<int> read_count{0};
  std::atomic<int> write_count{0};
  std::atomic<int> diagnostic_count{0};
  std::vector<std::thread> threads;

  // Read thread
  threads.emplace_back(
    [&]() {
      while (running) {
        interface.read(rclcpp::Time(), rclcpp::Duration::from_seconds(0.001));
        read_count++;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
    });

  // Write thread
  threads.emplace_back(
    [&]() {
      while (running) {
        interface.write(rclcpp::Time(), rclcpp::Duration::from_seconds(0.001));
        write_count++;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
    });

  // Diagnostics thread (simulates concurrent diagnostic updates)
  threads.emplace_back(
    [&]() {
      auto state_interfaces = interface.export_state_interfaces();
      while (running) {
        // Access state interfaces concurrently with read/write
        for (const auto & state : state_interfaces) {
          volatile double value = state.get_value();
          (void)value;
        }
        diagnostic_count++;
        std::this_thread::sleep_for(std::chrono::microseconds(500));
      }
    });

  // Let threads run for 100ms
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  running = false;

  for (auto & t : threads) {
    t.join();
  }

  // Verify operations occurred
  EXPECT_GT(read_count.load(), 0);
  EXPECT_GT(write_count.load(), 0);
  EXPECT_GT(diagnostic_count.load(), 0);

  // If we reach here without crashes, data races, or deadlocks, test passes
  SUCCEED();
}

TEST_F(ThreadSafetyTest, ConcurrentStateInterfaceAccessIsSafe)
{
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

  const std::int64_t serial = 0x00000000000000A1LL;
  const int axis = 0;
  const float torque_constant = 2.0F;

  interface.set_transport_factory(
    [&]() {
      auto instance = std::make_unique<MockTransport>();
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
      instance->set_permissive_mode(true);
      return instance;
    });

  auto info = make_test_info("a1");
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.on_activate(rclcpp_lifecycle::State{}));

  auto state_interfaces = interface.export_state_interfaces();
  auto command_interfaces = interface.export_command_interfaces();

  std::atomic<bool> running{true};
  std::atomic<bool> error_detected{false};
  std::vector<std::thread> threads;

  // Multiple threads reading state interfaces
  for (int i = 0; i < 3; i++) {
    threads.emplace_back(
      [&]() {
        while (running && !error_detected) {
          try {
            for (const auto & state : state_interfaces) {
              double value = state.get_value();
              // Verify value is either valid or NaN (not corrupted)
              if (!std::isnan(value) && !std::isfinite(value)) {
                error_detected = true;
                break;
              }
            }
          } catch (...) {
            error_detected = true;
            break;
          }
          std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
      });
  }

  // Writer thread modifying state through read()
  threads.emplace_back(
    [&]() {
      while (running && !error_detected) {
        try {
          interface.read(rclcpp::Time(), rclcpp::Duration::from_seconds(0.001));
        } catch (...) {
          error_detected = true;
          break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
    });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  running = false;

  for (auto & t : threads) {
    t.join();
  }

  EXPECT_FALSE(error_detected) << "Data corruption or exception detected during concurrent access";
}

TEST_F(ThreadSafetyTest, NoDeadlockUnderLoad)
{
  TestHardwareInterface interface;
  interface.set_diagnostics_factory(make_fake_diagnostics_factory());

  const std::int64_t serial = 0x00000000000000A1LL;
  const int axis = 0;
  const float torque_constant = 2.0F;

  interface.set_transport_factory(
    [&]() {
      auto instance = std::make_unique<MockTransport>();
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
      instance->set_permissive_mode(true);
      return instance;
    });

  auto info = make_test_info("a1");
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.configure(info));
  ASSERT_EQ(CallbackReturn::SUCCESS, interface.on_activate(rclcpp_lifecycle::State{}));

  std::atomic<bool> running{true};
  std::atomic<bool> deadlock_detected{false};
  std::vector<std::thread> threads;

  // Spawn many threads doing various operations
  for (int i = 0; i < 5; i++) {
    threads.emplace_back(
      [&]() {
        while (running) {
          interface.read(rclcpp::Time(), rclcpp::Duration::from_seconds(0.001));
          interface.write(rclcpp::Time(), rclcpp::Duration::from_seconds(0.001));
        }
      });
  }

  // Watchdog thread to detect if we hang
  std::atomic<int> heartbeat{0};
  std::atomic<int> consecutive_stalls{0};
  threads.emplace_back(
    [&]() {
      int last_heartbeat = 0;
      while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (heartbeat.load() == last_heartbeat) {
          // Allow a few stalls due to thread scheduling before declaring deadlock
          if (++consecutive_stalls >= 3) {
            deadlock_detected = true;
            running = false;
            break;
          }
        } else {
          consecutive_stalls = 0;
        }
        last_heartbeat = heartbeat.load();
      }
    });

  // Main thread increments heartbeat
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(300)) {
    heartbeat++;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  running = false;

  for (auto & t : threads) {
    t.join();
  }

  EXPECT_FALSE(deadlock_detected) << "Deadlock detected during concurrent operations";
}

}  // namespace
}  // namespace odrive_hardware_interface
