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

#include "odrive_hardware_interface/odrive_usb.hpp"

#include <cinttypes>
#include <cstdint>
#include <utility>

#include "rcutils/logging_macros.h"

namespace odrive
{
namespace
{
constexpr unsigned int kUsbTimeoutMs = 100;
constexpr std::uint16_t kMsbMask = 0x8000u;
constexpr std::uint16_t kSequenceMask = 0x7fffu;
constexpr const char kUsbLogger[] = "ODriveUSB";
}  // namespace

ODriveUSB::ODriveUSB()
: libusb_context_(nullptr),
  sequence_number_(0)
{
}

ODriveUSB::~ODriveUSB()
{
  while (!odrive_map_.empty()) {
    auto it = odrive_map_.begin();
    if (it->second != nullptr) {
      libusb_release_interface(it->second, 2);
      libusb_close(it->second);
    }
    odrive_map_.erase(it);
  }

  if (libusb_context_ != nullptr) {
    libusb_exit(libusb_context_);
    libusb_context_ = nullptr;
  }
}

int ODriveUSB::initialize(const SerialMatrix & serial_numbers)
{
  sequence_number_ = 0;
  int ret = libusb_init(&libusb_context_);
  if (ret != LIBUSB_SUCCESS) {
    return ret;
  }

  libusb_device ** device_list = nullptr;
  const ssize_t device_count = libusb_get_device_list(libusb_context_, &device_list);
  if (device_count <= 0) {
    libusb_free_device_list(device_list, 1);
    return static_cast<int>(device_count);
  }

  for (ssize_t i = 0; i < device_count; ++i) {
    libusb_device * device = device_list[i];
    libusb_device_descriptor descriptor;

    if (libusb_get_device_descriptor(device, &descriptor) != LIBUSB_SUCCESS) {
      continue;
    }

    if (
      descriptor.idVendor == ODRIVE_USB_VENDORID && descriptor.idProduct == ODRIVE_USB_PRODUCTID)
    {
      libusb_device_handle * device_handle = nullptr;
      if (libusb_open(device, &device_handle) != LIBUSB_SUCCESS) {
        continue;
      }
      if (
        (libusb_kernel_driver_active(device_handle, 2) != LIBUSB_SUCCESS) &&
        (libusb_detach_kernel_driver(device_handle, 2) != LIBUSB_SUCCESS))
      {
        libusb_close(device_handle);
        continue;
      }
      if (libusb_claim_interface(device_handle, 2) != LIBUSB_SUCCESS) {
        libusb_close(device_handle);
        continue;
      }
      std::uint64_t serial_number = 0;
      const int read_result = read(device_handle, SERIAL_NUMBER, serial_number);
      if (read_result != LIBUSB_SUCCESS) {
        libusb_release_interface(device_handle, 2);
        libusb_close(device_handle);
        continue;
      }
      odrive_map_.insert(
        std::pair<std::int64_t, libusb_device_handle *>(
          -static_cast<std::int64_t>(serial_number), device_handle));
    }
  }

  libusb_free_device_list(device_list, 1);
  if (odrive_map_.empty()) {
    return LIBUSB_ERROR_NO_DEVICE;
  }

  if (odrive_map_.size() == 1U) {
    auto it = odrive_map_.begin();
    odrive_map_.insert(std::pair<std::int64_t, libusb_device_handle *>(-it->first, it->second));
    RCUTILS_LOG_INFO_NAMED(
      kUsbLogger, "Connected to ODrive 0x%" PRIx64,
      static_cast<std::uint64_t>(-it->first));
    odrive_map_.erase(it);
  } else {
    for (std::size_t i = 0; i < serial_numbers.size(); ++i) {
      for (std::size_t j = 0; j < serial_numbers[i].size(); ++j) {
        const std::int64_t serial_number = serial_numbers[i][j];
        if (odrive_map_.count(serial_number) == 0U) {
          auto lookup = odrive_map_.find(-serial_number);
          if (lookup != odrive_map_.end()) {
            odrive_map_.insert(
              std::pair<std::int64_t, libusb_device_handle *>(
                -lookup->first, lookup->second));
            RCUTILS_LOG_INFO_NAMED(
              kUsbLogger, "Connected to ODrive 0x%" PRIx64,
              static_cast<std::uint64_t>(-lookup->first));
            odrive_map_.erase(lookup);
          } else {
            return LIBUSB_ERROR_NO_DEVICE;
          }
        }
      }
    }
  }

  for (auto it = odrive_map_.begin(); it != odrive_map_.end(); ) {
    if (it->first < 0) {
      if (it->second != nullptr) {
        libusb_release_interface(it->second, 2);
        libusb_close(it->second);
      }
      it = odrive_map_.erase(it);
    } else {
      ++it;
    }
  }

  return LIBUSB_SUCCESS;
}

template<typename T>
int ODriveUSB::read(libusb_device_handle * odrive_handle, std::int16_t endpoint_id, T & value)
{
  bytes request_payload;
  bytes response_payload;

  const int ret = endpointOperation(
    odrive_handle, endpoint_id, static_cast<std::int16_t>(sizeof(T)), request_payload,
    response_payload, true);
  if (ret != LIBUSB_SUCCESS) {
    RCUTILS_LOG_ERROR_NAMED(
      kUsbLogger, "Endpoint read error (endpoint %d): %s",
      static_cast<int>(endpoint_id),
      libusb_error_name(ret));
    return ret;
  }
  if (response_payload.size() < sizeof(T)) {
    RCUTILS_LOG_ERROR_NAMED(
      kUsbLogger,
      "Endpoint %d: received payload size %zu less than expected %zu",
      static_cast<int>(endpoint_id),
      response_payload.size(),
      sizeof(T));
    return LIBUSB_ERROR_IO;
  }
  std::memcpy(&value, response_payload.data(), sizeof(T));

  return LIBUSB_SUCCESS;
}

template<typename T>
int ODriveUSB::write(
  libusb_device_handle * odrive_handle, std::int16_t endpoint_id, const T & value)
{
  bytes request_payload;
  const auto * raw_value = reinterpret_cast<const uint8_t *>(&value);
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    request_payload.emplace_back(raw_value[i]);
  }
  bytes response_payload;
  return endpointOperation(odrive_handle, endpoint_id, 0, request_payload, response_payload, true);
}

int ODriveUSB::call(std::int64_t serial_number, std::int16_t endpoint_id)
{
  auto it = (serial_number != 0) ? odrive_map_.find(serial_number) : odrive_map_.begin();
  if (it == odrive_map_.end()) {
    return LIBUSB_ERROR_NO_DEVICE;
  }
  return call(it->second, endpoint_id);
}

int ODriveUSB::call(libusb_device_handle * odrive_handle, std::int16_t endpoint_id)
{
  bytes request_payload;
  bytes response_payload;
  return endpointOperation(odrive_handle, endpoint_id, 0, request_payload, response_payload, true);
}

int ODriveUSB::endpointOperation(
  libusb_device_handle * odrive_handle, std::int16_t endpoint_id, std::int16_t response_size,
  bytes request_payload, bytes & response_payload, bool expect_response)
{
  int transferred = 0;
  bytes response_packet;
  unsigned char response_data[ODRIVE_MAX_PACKET_SIZE] = {0};

  std::int16_t effective_endpoint = endpoint_id;
  if (expect_response) {
    effective_endpoint = static_cast<std::int16_t>(effective_endpoint | kMsbMask);
  }
  sequence_number_ = static_cast<std::int16_t>((sequence_number_ + 1) & kSequenceMask);
  sequence_number_ = static_cast<std::int16_t>(sequence_number_ | LIBUSB_ENDPOINT_IN);
  const std::int16_t sequence_number = sequence_number_;

  bytes request_packet = encodePacket(
    sequence_number, effective_endpoint, response_size, request_payload);

  int ret = libusb_bulk_transfer(
    odrive_handle, ODRIVE_OUT_ENDPOINT, request_packet.data(), request_packet.size(), &transferred,
    kUsbTimeoutMs);
  if (ret != LIBUSB_SUCCESS) {
    RCUTILS_LOG_ERROR_NAMED(
      kUsbLogger,
      "Bulk transfer failed on OUT endpoint (seq: %d, endpoint: %d): %s",
      static_cast<int>(sequence_number),
      static_cast<int>(effective_endpoint),
      libusb_error_name(ret));
    return ret;
  }

  if (expect_response) {
    ret = libusb_bulk_transfer(
      odrive_handle, ODRIVE_IN_ENDPOINT, response_data, ODRIVE_MAX_PACKET_SIZE, &transferred,
      kUsbTimeoutMs);
    if (ret != LIBUSB_SUCCESS) {
      RCUTILS_LOG_ERROR_NAMED(
        kUsbLogger,
        "Bulk transfer failed on IN endpoint (seq: %d, endpoint: %d): %s",
        static_cast<int>(sequence_number),
        static_cast<int>(effective_endpoint),
        libusb_error_name(ret));
      return ret;
    }
    if (transferred < response_size) {
      RCUTILS_LOG_WARN_NAMED(
        kUsbLogger,
        "Bulk transfer (seq: %d, endpoint: %d): transferred %d bytes, expected %d",
        static_cast<int>(sequence_number),
        static_cast<int>(effective_endpoint),
        transferred,
        static_cast<int>(response_size));
      return LIBUSB_ERROR_IO;
    }
    for (int i = 0; i < transferred; ++i) {
      response_packet.emplace_back(response_data[i]);
    }
    response_payload = decodePacket(response_packet);
  }

  return LIBUSB_SUCCESS;
}

bytes ODriveUSB::encodePacket(
  std::int16_t sequence_number, std::int16_t endpoint_id, std::int16_t response_size,
  const bytes & request_payload)
{
  bytes packet;
  packet.emplace_back(static_cast<uint8_t>((sequence_number >> 0) & 0xFF));
  packet.emplace_back(static_cast<uint8_t>((sequence_number >> 8) & 0xFF));
  packet.emplace_back(static_cast<uint8_t>((endpoint_id >> 0) & 0xFF));
  packet.emplace_back(static_cast<uint8_t>((endpoint_id >> 8) & 0xFF));
  packet.emplace_back(static_cast<uint8_t>((response_size >> 0) & 0xFF));
  packet.emplace_back(static_cast<uint8_t>((response_size >> 8) & 0xFF));

  for (uint8_t b : request_payload) {
    packet.emplace_back(b);
  }

  const std::int16_t crc = ((endpoint_id & 0x7fff) == 0) ? ODRIVE_PROTOCOL_VERSION : json_crc;
  packet.emplace_back(static_cast<uint8_t>((crc >> 0) & 0xFF));
  packet.emplace_back(static_cast<uint8_t>((crc >> 8) & 0xFF));

  return packet;
}

bytes ODriveUSB::decodePacket(bytes & response_packet)
{
  bytes payload;
  for (bytes::size_type i = 2; i < response_packet.size(); ++i) {
    payload.emplace_back(response_packet[i]);
  }
  return payload;
}

int ODriveUSB::read_impl(
  std::int64_t serial_number, std::int16_t endpoint_id, void * value, std::size_t size)
{
  auto it = (serial_number != 0) ? odrive_map_.find(serial_number) : odrive_map_.begin();
  if (it == odrive_map_.end()) {
    return LIBUSB_ERROR_NO_DEVICE;
  }
  bytes request_payload;
  bytes response_payload;
  const int ret = endpointOperation(
    it->second, endpoint_id, static_cast<std::int16_t>(size), request_payload, response_payload,
    true);
  if (ret != LIBUSB_SUCCESS) {
    return ret;
  }
  if (response_payload.size() < size) {
    return LIBUSB_ERROR_IO;
  }
  std::memcpy(value, response_payload.data(), size);
  return LIBUSB_SUCCESS;
}

int ODriveUSB::write_impl(
  std::int64_t serial_number, std::int16_t endpoint_id, const void * value, std::size_t size)
{
  auto it = (serial_number != 0) ? odrive_map_.find(serial_number) : odrive_map_.begin();
  if (it == odrive_map_.end()) {
    return LIBUSB_ERROR_NO_DEVICE;
  }
  bytes request_payload;
  const auto * raw_value = reinterpret_cast<const uint8_t *>(value);
  for (std::size_t i = 0; i < size; ++i) {
    request_payload.emplace_back(raw_value[i]);
  }
  bytes response_payload;
  return endpointOperation(it->second, endpoint_id, 0, request_payload, response_payload, true);
}

template int ODriveUSB::read<bool>(libusb_device_handle *, std::int16_t, bool &);
template int ODriveUSB::read<float>(libusb_device_handle *, std::int16_t, float &);
template int ODriveUSB::read<int32_t>(libusb_device_handle *, std::int16_t, int32_t &);
template int ODriveUSB::read<uint8_t>(libusb_device_handle *, std::int16_t, uint8_t &);
template int ODriveUSB::read<uint16_t>(libusb_device_handle *, std::int16_t, uint16_t &);
template int ODriveUSB::read<uint32_t>(libusb_device_handle *, std::int16_t, uint32_t &);
template int ODriveUSB::read<uint64_t>(libusb_device_handle *, std::int16_t, uint64_t &);

template int ODriveUSB::write<bool>(libusb_device_handle *, std::int16_t, const bool &);
template int ODriveUSB::write<float>(libusb_device_handle *, std::int16_t, const float &);
template int ODriveUSB::write<int32_t>(libusb_device_handle *, std::int16_t, const int32_t &);
template int ODriveUSB::write<uint8_t>(libusb_device_handle *, std::int16_t, const uint8_t &);
template int ODriveUSB::write<uint16_t>(libusb_device_handle *, std::int16_t, const uint16_t &);
template int ODriveUSB::write<uint32_t>(libusb_device_handle *, std::int16_t, const uint32_t &);
template int ODriveUSB::write<uint64_t>(libusb_device_handle *, std::int16_t, const uint64_t &);

}  // namespace odrive
