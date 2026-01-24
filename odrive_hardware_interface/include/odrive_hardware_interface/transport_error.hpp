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

#include <cerrno>
#include <cstdint>

namespace odrive_hardware_interface
{

/// Type-safe error codes for ODrive transport operations.
/// Replaces raw errno codes with explicit, documented error types.
///
/// ERROR CODE REFERENCE:
/// ┌──────────────────────────┬────────┬──────────────┬────────────────┐
/// │ Error Type               │ Code   │ Retryable    │ Fatal          │
/// ├──────────────────────────┼────────┼──────────────┼────────────────┤
/// │ OK                       │ 0      │ N/A          │ No             │
/// │ DEVICE_NOT_FOUND         │ -19    │ No           │ Yes            │
/// │ PERMISSION_DENIED        │ -1     │ No           │ Yes            │
/// │ INVALID_PARAMETER        │ -22    │ No           │ Yes            │
/// │ TIMEOUT                  │ -110   │ Yes          │ No             │
/// │ BUSY                     │ -16    │ Yes          │ No             │
/// │ IO_ERROR                 │ -5     │ Yes          │ No             │
/// │ PROTOCOL_ERROR           │ -71    │ No           │ No             │
/// │ ENDPOINT_NOT_SUPPORTED   │ -95    │ No           │ No             │
/// │ UNKNOWN                  │ -999   │ No           │ No             │
/// └──────────────────────────┴────────┴──────────────┴────────────────┘
///
/// TROUBLESHOOTING:
/// - DEVICE_NOT_FOUND (-19): Check USB cable, run 'lsusb' to verify device
/// - PERMISSION_DENIED (-1): Add udev rules, add user to 'dialout' group
/// - TIMEOUT (-110): Reduce usb_timeout_ms or check for EMI/cable quality
/// - BUSY (-16): Device in use by another process (check for odrivetool)
/// - IO_ERROR (-5): Intermittent USB issue, cable problem, or power glitch
enum class TransportError : int
{
  /// Operation completed successfully
  OK = 0,

  /// Device not found or disconnected (-ENODEV)
  DEVICE_NOT_FOUND = -19,

  /// Permission denied, typically USB access rights (-EPERM)
  PERMISSION_DENIED = -1,

  /// Invalid parameter or endpoint (-EINVAL)
  INVALID_PARAMETER = -22,

  /// Operation timed out (-ETIMEDOUT)
  TIMEOUT = -110,

  /// Device or resource busy (-EBUSY)
  BUSY = -16,

  /// I/O error during communication (-EIO)
  IO_ERROR = -5,

  /// Protocol-level error (data corruption, CRC mismatch, etc.)
  PROTOCOL_ERROR = -71,

  /// Endpoint not supported by firmware
  ENDPOINT_NOT_SUPPORTED = -95,

  /// Unknown or unmapped error
  UNKNOWN = -999
};

/// Convert errno-style error code to TransportError enum.
inline TransportError errno_to_transport_error(int err_code)
{
  if (err_code == 0) {
    return TransportError::OK;
  }
  if (err_code == -ENODEV) {
    return TransportError::DEVICE_NOT_FOUND;
  }
  if (err_code == -EPERM) {
    return TransportError::PERMISSION_DENIED;
  }
  if (err_code == -EINVAL) {
    return TransportError::INVALID_PARAMETER;
  }
  if (err_code == -ETIMEDOUT) {
    return TransportError::TIMEOUT;
  }
  if (err_code == -EBUSY) {
    return TransportError::BUSY;
  }
  if (err_code == -EIO) {
    return TransportError::IO_ERROR;
  }
  if (err_code == -EPROTO) {
    return TransportError::PROTOCOL_ERROR;
  }
  if (err_code == -EOPNOTSUPP) {
    return TransportError::ENDPOINT_NOT_SUPPORTED;
  }
  return TransportError::UNKNOWN;
}

/// Convert TransportError to raw error code for legacy compatibility.
inline int to_error_code(TransportError err)
{
  return static_cast<int>(err);
}

/// Get human-readable description of transport error.
inline const char * to_string(TransportError err)
{
  switch (err) {
    case TransportError::OK:
      return "Success";
    case TransportError::DEVICE_NOT_FOUND:
      return "Device not found or disconnected";
    case TransportError::PERMISSION_DENIED:
      return "Permission denied (check USB permissions)";
    case TransportError::INVALID_PARAMETER:
      return "Invalid parameter or endpoint";
    case TransportError::TIMEOUT:
      return "Operation timed out";
    case TransportError::BUSY:
      return "Device or resource busy";
    case TransportError::IO_ERROR:
      return "I/O error during communication";
    case TransportError::PROTOCOL_ERROR:
      return "Protocol error (data corruption or CRC mismatch)";
    case TransportError::ENDPOINT_NOT_SUPPORTED:
      return "Endpoint not supported by firmware";
    case TransportError::UNKNOWN:
    default:
      return "Unknown error";
  }
}

/// Check if error is transient and worth retrying.
inline bool is_retryable(TransportError err)
{
  switch (err) {
    case TransportError::TIMEOUT:
    case TransportError::BUSY:
    case TransportError::IO_ERROR:
      return true;
    case TransportError::DEVICE_NOT_FOUND:
    case TransportError::PERMISSION_DENIED:
    case TransportError::INVALID_PARAMETER:
    case TransportError::PROTOCOL_ERROR:
    case TransportError::ENDPOINT_NOT_SUPPORTED:
    case TransportError::OK:
    case TransportError::UNKNOWN:
    default:
      return false;
  }
}

/// Check if error indicates device is permanently unavailable.
inline bool is_fatal(TransportError err)
{
  switch (err) {
    case TransportError::DEVICE_NOT_FOUND:
    case TransportError::PERMISSION_DENIED:
    case TransportError::INVALID_PARAMETER:
      return true;
    default:
      return false;
  }
}

}  // namespace odrive_hardware_interface
