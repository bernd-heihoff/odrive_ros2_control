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

#ifndef ODRIVE_HARDWARE_INTERFACE__ODRIVE_TRANSPORT_HPP_
#define ODRIVE_HARDWARE_INTERFACE__ODRIVE_TRANSPORT_HPP_

#include <cstdint>
#include <vector>

#include "odrive_hardware_interface/visibility_control.hpp"

namespace odrive_hardware_interface
{
/// Abstract interface for ODrive USB communication.
///
/// This interface abstracts the low-level USB protocol used by ODrive controllers,
/// allowing for dependency injection and testing with mock transports.
///
/// Thread Safety: Implementations should be thread-safe for concurrent read/write
/// operations from the same or different threads.
///
/// Error Handling: All methods return 0 on success, negative error codes on failure.
/// Error codes typically match errno values (e.g., -ENODEV, -ETIMEDOUT).
class ODRIVE_HARDWARE_INTERFACE_PUBLIC ODriveTransport
{
public:
  using SerialMatrix = std::vector<std::vector<std::int64_t>>;

  virtual ~ODriveTransport() = default;

  /// Initialize transport and discover ODrive devices.
  ///
  /// @param serial_numbers Matrix of serial numbers organized by bus/device hierarchy.
  ///                       Typically a 2D vector where each row represents devices on a bus.
  /// @return 0 on success, negative error code on failure
  virtual int initialize(const SerialMatrix & serial_numbers) = 0;

  /// Read a value from an ODrive endpoint.
  ///
  /// Template method that reads sizeof(T) bytes from the specified endpoint
  /// and interprets them as type T.
  ///
  /// @tparam T Value type (must match endpoint's data type)
  /// @param serial_number ODrive serial number in decimal format
  /// @param endpoint_id Endpoint ID from odrive_endpoints.hpp
  /// @param[out] value Read value on success
  /// @return 0 on success, negative error code on failure
  template<typename T>
  int read(std::int64_t serial_number, std::int16_t endpoint_id, T & value)
  {
    return read_impl(serial_number, endpoint_id, &value, sizeof(T));
  }

  /// Write a value to an ODrive endpoint.
  ///
  /// Template method that writes sizeof(T) bytes to the specified endpoint.
  ///
  /// @tparam T Value type (must match endpoint's data type)
  /// @param serial_number ODrive serial number in decimal format
  /// @param endpoint_id Endpoint ID from odrive_endpoints.hpp
  /// @param value Value to write
  /// @return 0 on success, negative error code on failure
  template<typename T>
  int write(std::int64_t serial_number, std::int16_t endpoint_id, const T & value)
  {
    return write_impl(serial_number, endpoint_id, &value, sizeof(T));
  }

  /// Call a void endpoint (function with no return value).
  ///
  /// Used for endpoints that trigger actions without returning data,
  /// such as AXIS__WATCHDOG_FEED or AXIS__CLEAR_ERRORS.
  ///
  /// @param serial_number ODrive serial number in decimal format
  /// @param endpoint_id Endpoint ID from odrive_endpoints.hpp
  /// @return 0 on success, negative error code on failure
  virtual int call(std::int64_t serial_number, std::int16_t endpoint_id) = 0;

protected:
  virtual int read_impl(
    std::int64_t serial_number, std::int16_t endpoint_id, void * value, std::size_t size) = 0;

  virtual int write_impl(
    std::int64_t serial_number, std::int16_t endpoint_id, const void * value, std::size_t size) = 0;
};
}  // namespace odrive_hardware_interface

#endif  // ODRIVE_HARDWARE_INTERFACE__ODRIVE_TRANSPORT_HPP_
