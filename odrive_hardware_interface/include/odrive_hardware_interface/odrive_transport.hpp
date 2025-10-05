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
class ODRIVE_HARDWARE_INTERFACE_PUBLIC ODriveTransport
{
public:
  using SerialMatrix = std::vector<std::vector<std::int64_t>>;

  virtual ~ODriveTransport() = default;

  virtual int initialize(const SerialMatrix & serial_numbers) = 0;

  template<typename T>
  int read(std::int64_t serial_number, std::int16_t endpoint_id, T & value)
  {
    return read_impl(serial_number, endpoint_id, &value, sizeof(T));
  }

  template<typename T>
  int write(std::int64_t serial_number, std::int16_t endpoint_id, const T & value)
  {
    return write_impl(serial_number, endpoint_id, &value, sizeof(T));
  }

  virtual int call(std::int64_t serial_number, std::int16_t endpoint_id) = 0;

protected:
  virtual int read_impl(
    std::int64_t serial_number, std::int16_t endpoint_id, void * value, std::size_t size) = 0;

  virtual int write_impl(
    std::int64_t serial_number, std::int16_t endpoint_id, const void * value, std::size_t size) = 0;
};
}  // namespace odrive_hardware_interface

#endif  // ODRIVE_HARDWARE_INTERFACE__ODRIVE_TRANSPORT_HPP_
