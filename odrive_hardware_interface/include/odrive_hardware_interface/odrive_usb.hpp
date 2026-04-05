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

#include <libusb-1.0/libusb.h>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <atomic>
#include <vector>

#include "odrive_hardware_interface/odrive_endpoints.hpp"
#include "odrive_hardware_interface/odrive_transport.hpp"

#define ODRIVE_USB_VENDORID 0x1209
#define ODRIVE_USB_PRODUCTID 0x0d32

#define ODRIVE_OUT_ENDPOINT 0x03
#define ODRIVE_IN_ENDPOINT 0x83

#define ODRIVE_PROTOCOL_VERSION 1
#define ODRIVE_MAX_PACKET_SIZE 16

using bytes = std::vector<uint8_t>;

namespace odrive
{
class ODriveUSB : public odrive_hardware_interface::ODriveTransport
{
public:
  ODriveUSB();
  ~ODriveUSB();

  void set_timeout_ms(unsigned int timeout_ms) override;

  int initialize(const SerialMatrix & serial_numbers) override;
  int call(std::int64_t serial_number, std::int16_t endpoint_id) override;

private:
  libusb_context * libusb_context_;

  std::map<std::int64_t, libusb_device_handle *> odrive_map_;

  std::int16_t sequence_number_;

  std::atomic<int> sticky_error_{0};
  unsigned int usb_timeout_ms_{100};

  template<typename T>
  int read(libusb_device_handle * odrive_handle, std::int16_t endpoint_id, T & value);
  template<typename T>
  int write(
    libusb_device_handle * odrive_handle, std::int16_t endpoint_id, const T & value);
  int call(libusb_device_handle * odrive_handle, std::int16_t endpoint_id);

  int endpointOperation(
    libusb_device_handle * odrive_handle, std::int16_t endpoint_id, std::int16_t response_size,
    bytes request_payload, bytes & response_payload, bool MSB);

  bytes encodePacket(
    std::int16_t sequence_number, std::int16_t endpoint_id, std::int16_t response_size,
    const bytes & request_payload);
  bytes decodePacket(bytes & response_packet);

  int read_impl(
    std::int64_t serial_number, std::int16_t endpoint_id, void * value,
    std::size_t size) override;
  int write_impl(
    std::int64_t serial_number, std::int16_t endpoint_id, const void * value,
    std::size_t size) override;
};
}  // namespace odrive
