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

#include <cstdint>
#include <string>

namespace odrive_hardware_interface
{

/// Type-safe transport error codes
enum class TransportError : std::int32_t
{
  OK = 0,

  // libusb error codes (matching libusb_error enum)
  USB_IO_ERROR = -1,
  USB_INVALID_PARAM = -2,
  USB_ACCESS_DENIED = -3,
  USB_NO_DEVICE = -4,
  USB_NOT_FOUND = -5,
  USB_BUSY = -6,
  USB_TIMEOUT = -7,
  USB_OVERFLOW = -8,
  USB_PIPE = -9,
  USB_INTERRUPTED = -10,
  USB_NO_MEM = -11,
  USB_NOT_SUPPORTED = -12,
  USB_OTHER = -99,

  // Custom application errors
  DEVICE_NOT_FOUND = -100,
  INVALID_PARAMETER = -101,
  ENDPOINT_MISMATCH = -102,
  BUFFER_TOO_SMALL = -103,
  PROTOCOL_ERROR = -104,
};

/// Convert TransportError to double for state interface exposure
inline double to_state_value(TransportError error)
{
  return static_cast<double>(static_cast<std::int32_t>(error));
}

/// Convert int status code to TransportError enum
inline TransportError from_status_code(int status)
{
  if (status == 0) {
    return TransportError::OK;
  }

  // Map common libusb errors
  switch (status) {
    case -1: return TransportError::USB_IO_ERROR;
    case -2: return TransportError::USB_INVALID_PARAM;
    case -3: return TransportError::USB_ACCESS_DENIED;
    case -4: return TransportError::USB_NO_DEVICE;
    case -5: return TransportError::USB_NOT_FOUND;
    case -6: return TransportError::USB_BUSY;
    case -7: return TransportError::USB_TIMEOUT;
    case -8: return TransportError::USB_OVERFLOW;
    case -9: return TransportError::USB_PIPE;
    case -10: return TransportError::USB_INTERRUPTED;
    case -11: return TransportError::USB_NO_MEM;
    case -12: return TransportError::USB_NOT_SUPPORTED;
    default: return TransportError::USB_OTHER;
  }
}

/// Get human-readable description of error code
inline std::string describe_error(TransportError error)
{
  switch (error) {
    case TransportError::OK: return "OK";
    case TransportError::USB_IO_ERROR: return "USB I/O error";
    case TransportError::USB_INVALID_PARAM: return "Invalid parameter";
    case TransportError::USB_ACCESS_DENIED: return "Access denied";
    case TransportError::USB_NO_DEVICE: return "No such device";
    case TransportError::USB_NOT_FOUND: return "Entity not found";
    case TransportError::USB_BUSY: return "Resource busy";
    case TransportError::USB_TIMEOUT: return "Operation timeout";
    case TransportError::USB_OVERFLOW: return "Overflow";
    case TransportError::USB_PIPE: return "Pipe error";
    case TransportError::USB_INTERRUPTED: return "System call interrupted";
    case TransportError::USB_NO_MEM: return "Insufficient memory";
    case TransportError::USB_NOT_SUPPORTED: return "Operation not supported";
    case TransportError::USB_OTHER: return "Other USB error";
    case TransportError::DEVICE_NOT_FOUND: return "Device not found";
    case TransportError::INVALID_PARAMETER: return "Invalid parameter";
    case TransportError::ENDPOINT_MISMATCH: return "Endpoint mismatch";
    case TransportError::BUFFER_TOO_SMALL: return "Buffer too small";
    case TransportError::PROTOCOL_ERROR: return "Protocol error";
    default: return "Unknown error";
  }
}

}  // namespace odrive_hardware_interface
