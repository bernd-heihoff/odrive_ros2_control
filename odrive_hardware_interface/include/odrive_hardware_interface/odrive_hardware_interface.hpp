// Copyright 2021 Factor Robotics
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

#include <chrono>
#include <cmath>
#include <cstdint>
#include <atomic>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_hardware_interface/axis_control.hpp"
#include "odrive_hardware_interface/diagnostics_interface.hpp"
#include "odrive_hardware_interface/odrive_configuration.hpp"
#include "odrive_hardware_interface/odrive_transport.hpp"
#include "odrive_hardware_interface/visibility_control.hpp"
#include "rclcpp/rclcpp.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace diagnostic_updater
{
class DiagnosticStatusWrapper;
class Updater;
}  // namespace diagnostic_updater

namespace odrive_hardware_interface
{
class ODriveHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ODriveHardwareInterface)

  ODriveHardwareInterface();

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  ~ODriveHardwareInterface() override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  return_type perform_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> &) override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn recover();

  using TransportFactory = std::function<std::unique_ptr<ODriveTransport>()>;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  void set_transport_factory(TransportFactory factory);

  using DiagnosticsFactory = odrive_hardware_interface::DiagnosticsFactory;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  void set_diagnostics_factory(DiagnosticsFactory factory);

protected:
  CallbackReturn configure_from_info(const hardware_interface::HardwareInfo & info);

private:
  friend class DiagnosticsTestHelper;
  friend class DiagnosticsConfigTestHelper;

  struct SafetyConfig
  {
    // When true, drive commands are only sent while the hardware component is ACTIVE.
    bool gate_outputs_with_lifecycle{true};
    // When true, faulted axes are masked (commands skipped) so remaining axes can keep operating.
    bool mask_faulted_axes{true};
    // When true, a faulted axis is transitioned to IDLE once when a fault is detected.
    bool request_idle_on_axis_fault{true};
  };

  struct RuntimeConfig
  {
    // TIMEOUT DESIGN RATIONALE:
    // All timeouts are FIXED (not adaptive) to maintain deterministic real-time behavior.
    // In a control loop, we need predictable worst-case execution time. Adaptive timeouts
    // would hide problems (bad cables, EMI) and make debugging harder. If communication
    // fails consistently, the system should fault fast and surface the issue rather than
    // progressively degrading performance with longer retry periods.

    // USB communication timeout in milliseconds
    unsigned int usb_timeout_ms{100};
    // Throttle interval for repeated log messages in milliseconds
    unsigned int log_throttle_ms{2000};
    // Retry logic for transient USB errors (fixed retry count for deterministic timing)
    unsigned int usb_read_retries{3};
    unsigned int usb_write_retries{3};
    unsigned int usb_retry_delay_ms{5};
    // Deadline monitoring
    double max_read_cycle_time_sec{0.010};   // 10ms
    double max_write_cycle_time_sec{0.010};  // 10ms
    bool enable_deadline_warnings{true};
  };

  CallbackReturn initialize_transport();
  void reset_runtime_state();
  void register_diagnostics_tasks();
  void populate_joint_diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper & status,
    std::size_t index) const;
  void populate_drive_diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper & status,
    std::size_t index) const;
  void populate_sensor_diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper & status,
    std::size_t index) const;
  void maybe_update_diagnostics();
  static std::string serial_to_hex(std::int64_t serial_number);
  static std::string join_messages(const std::vector<std::string> & parts);

  struct SensorContext
  {
    std::int64_t serial_number{0};
    double vbus_voltage{std::numeric_limits<double>::quiet_NaN()};
    // 0.0 means OK; non-zero holds the last transport error code.
    double transport_error{0.0};
  };

  struct DriveContext
  {
    std::int64_t serial_number{0};
    std::string label;

    // ODrive CAN interface error bitfield (odrivetool: odrv0.can.error).
    std::uint32_t can_error{0};
    // 0 means OK; non-zero holds the last transport error code while reading can_error.
    double can_error_read_error{0.0};
  };

  struct JointContext
  {
    std::int64_t serial_number{0};
    int axis{0};
    bool invert_axis{false};  // Invert position/velocity/effort signs
    float torque_constant{std::numeric_limits<float>::quiet_NaN()};

    // Previous command values for rate limiting
    double last_command_position{std::numeric_limits<double>::quiet_NaN()};
    double last_command_velocity{std::numeric_limits<double>::quiet_NaN()};
    double last_command_effort{std::numeric_limits<double>::quiet_NaN()};

    // Previous feedback values for discontinuity detection
    double last_valid_position{std::numeric_limits<double>::quiet_NaN()};
    double last_valid_velocity{std::numeric_limits<double>::quiet_NaN()};

    // WATCHDOG SAFETY DESIGN:
    // Watchdog feeds occur ONLY during write() when commands are actively sent.
    // This is intentional: the watchdog should detect control loop failures,
    // not just communication failures. If the controller stops commanding
    // (lifecycle gating, faulted axis masking, etc.), the watchdog SHOULD expire
    // to safely halt the motor. Set watchdog_timeout conservatively above your
    // worst-case control loop period.
    bool enable_watchdog{false};

    double command_position{std::numeric_limits<double>::quiet_NaN()};
    double command_velocity{std::numeric_limits<double>::quiet_NaN()};
    double command_effort{std::numeric_limits<double>::quiet_NaN()};

    double position{std::numeric_limits<double>::quiet_NaN()};
    double velocity{std::numeric_limits<double>::quiet_NaN()};
    double effort{std::numeric_limits<double>::quiet_NaN()};

    double axis_error{std::numeric_limits<double>::quiet_NaN()};
    double motor_error{std::numeric_limits<double>::quiet_NaN()};
    double encoder_error{std::numeric_limits<double>::quiet_NaN()};
    double controller_error{std::numeric_limits<double>::quiet_NaN()};
    double fet_temperature{std::numeric_limits<double>::quiet_NaN()};
    double motor_temperature{std::numeric_limits<double>::quiet_NaN()};

    // 0.0 means OK; non-zero holds the last transport error code.
    double read_error{0.0};
    // 0.0 means OK; non-zero holds the last transport error code.
    double write_error{0.0};
    // 1.0 means the latest telemetry sample is complete; 0.0 otherwise.
    double telemetry_valid{0.0};

    // 1.0 means axis telemetry read succeeded and all axis error registers were 0; 0.0 otherwise.
    double healthy{0.0};

    // Rate limiter state (1.0 when limited, 0.0 otherwise)
    double position_rate_limited{0.0};
    double velocity_rate_limited{0.0};
    double effort_rate_limited{0.0};

    AxisControlLevel control_level{AxisControlLevel::UNDEFINED};
    JointConfig::CommandLimits command_limits;
    std::uint64_t last_axis_error{0};
    std::uint64_t last_motor_error{0};
    std::uint64_t last_encoder_error{0};
    std::uint64_t last_controller_error{0};

    JointConfig::FeedbackLimits feedback_limits;
  };

  static void hold_joint_state(JointContext & joint);

  TransportFactory transport_factory_;
  DiagnosticsFactory diagnostics_factory_;
  HardwareConfiguration hardware_config_;
  std::unique_ptr<ODriveTransport> transport_;

  SafetyConfig safety_config_;
  RuntimeConfig runtime_config_;
  // Atomic flag for lock-free lifecycle state checks in write path
  std::atomic<bool> outputs_enabled_{false};

  // Runtime tracking for axis fault masking.
  std::vector<bool> axis_faulted_;
  std::vector<bool> idle_requested_on_fault_;

  struct DiagnosticsConfig
  {
    bool enabled{true};
    double period_sec{0.5};
    double warn_temperature_deg_c{85.0};
    double error_temperature_deg_c{95.0};
  } diagnostics_config_;

  // Performance monitoring statistics
  struct CycleStats
  {
    std::size_t read_cycles{0};
    std::size_t write_cycles{0};
    std::size_t read_deadline_misses{0};
    std::size_t write_deadline_misses{0};
    double max_read_cycle_time_sec{0.0};
    double max_write_cycle_time_sec{0.0};
    std::size_t usb_read_retries_total{0};
    std::size_t usb_write_retries_total{0};
    std::size_t command_validation_failures{0};
    std::size_t rate_limit_events{0};
  } cycle_stats_;

  std::shared_ptr<DiagnosticsInterface> diagnostics_;
  DiagnosticsInterface::Duration diagnostics_period_{DiagnosticsInterface::Duration::zero()};
  std::optional<DiagnosticsInterface::TimePoint> last_diagnostics_update_;

  // LOCKING STRATEGY:
  // - state_mutex_: Primary shared mutex using reader-writer semantics
  //   * read() and diagnostics use shared locks (concurrent reads allowed)
  //   * write() uses exclusive lock (blocks all other access)
  // - Per-context fine-grained locks would further reduce contention, but add
  //   complexity. Current design prioritizes correctness and simplicity.
  // - outputs_enabled_ is atomic for lock-free lifecycle state checks
  //
  // Lock ordering (when multiple locks needed): state_mutex_ must be acquired first
  mutable std::shared_mutex state_mutex_;

  std::vector<SensorContext> sensors_;
  std::vector<DriveContext> drives_;
  std::vector<JointContext> joints_;
};
}  // namespace odrive_hardware_interface
