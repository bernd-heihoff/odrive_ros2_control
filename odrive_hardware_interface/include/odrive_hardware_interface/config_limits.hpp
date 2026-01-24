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

namespace odrive_hardware_interface
{
namespace config_limits
{

// USB Communication Timeouts
/// Minimum USB timeout (1ms prevents immediate failure)
constexpr unsigned int MIN_USB_TIMEOUT_MS = 1;
/// Maximum USB timeout (10s prevents indefinite hangs)
constexpr unsigned int MAX_USB_TIMEOUT_MS = 10000;
/// Default USB timeout (100ms balances responsiveness and reliability)
constexpr unsigned int DEFAULT_USB_TIMEOUT_MS = 100;

// Logging Configuration
/// Minimum log throttle interval (100ms prevents console spam)
constexpr unsigned int MIN_LOG_THROTTLE_MS = 100;
/// Maximum log throttle interval (60s ensures timely error reporting)
constexpr unsigned int MAX_LOG_THROTTLE_MS = 60000;
/// Default log throttle interval (2s balances visibility and noise)
constexpr unsigned int DEFAULT_LOG_THROTTLE_MS = 2000;

// USB Retry Logic
/// Default number of retries for transient read errors
constexpr unsigned int DEFAULT_USB_READ_RETRIES = 3;
/// Default number of retries for transient write errors
constexpr unsigned int DEFAULT_USB_WRITE_RETRIES = 3;
/// Delay between retry attempts (5ms allows device recovery)
constexpr unsigned int DEFAULT_USB_RETRY_DELAY_MS = 5;

// Real-time Deadline Monitoring
/// Maximum acceptable read cycle time (10ms for 100Hz control loops)
constexpr double DEFAULT_MAX_READ_CYCLE_TIME_SEC = 0.010;
/// Maximum acceptable write cycle time (10ms for 100Hz control loops)
constexpr double DEFAULT_MAX_WRITE_CYCLE_TIME_SEC = 0.010;

// Diagnostics Configuration
/// Minimum diagnostics period (100ms, warning if lower for RT performance)
constexpr double MIN_DIAGNOSTIC_PERIOD_SEC = 0.1;
/// Default diagnostics period (500ms balances overhead and freshness)
constexpr double DEFAULT_DIAGNOSTIC_PERIOD_SEC = 0.5;
/// Default warning temperature threshold (85°C for MOSFETs)
constexpr double DEFAULT_WARN_TEMPERATURE_DEG_C = 85.0;
/// Default error temperature threshold (95°C approaching max ratings)
constexpr double DEFAULT_ERROR_TEMPERATURE_DEG_C = 95.0;

}  // namespace config_limits
}  // namespace odrive_hardware_interface
