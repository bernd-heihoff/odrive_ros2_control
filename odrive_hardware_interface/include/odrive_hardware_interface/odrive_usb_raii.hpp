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

#include <libusb-1.0/libusb.h>
#include <utility>

namespace odrive
{

/// RAII wrapper for libusb_context to ensure proper cleanup
class LibUsbContext
{
public:
  LibUsbContext()
  : ctx_(nullptr) {}

  ~LibUsbContext()
  {
    if (ctx_ != nullptr) {
      libusb_exit(ctx_);
      ctx_ = nullptr;
    }
  }

  // No copy
  LibUsbContext(const LibUsbContext &) = delete;
  LibUsbContext & operator=(const LibUsbContext &) = delete;

  // Move allowed
  LibUsbContext(LibUsbContext && other) noexcept
  : ctx_(other.ctx_)
  {
    other.ctx_ = nullptr;
  }

  LibUsbContext & operator=(LibUsbContext && other) noexcept
  {
    if (this != &other) {
      if (ctx_ != nullptr) {
        libusb_exit(ctx_);
      }
      ctx_ = other.ctx_;
      other.ctx_ = nullptr;
    }
    return *this;
  }

  int init()
  {
    return libusb_init(&ctx_);
  }

  libusb_context * get() const
  {
    return ctx_;
  }

  explicit operator bool() const
  {
    return ctx_ != nullptr;
  }

private:
  libusb_context * ctx_;
};

/// RAII wrapper for libusb_device_handle to ensure proper cleanup
class LibUsbDevice
{
public:
  LibUsbDevice()
  : handle_(nullptr) {}

  explicit LibUsbDevice(libusb_device_handle * h)
  : handle_(h)
  {
  }

  ~LibUsbDevice()
  {
    reset();
  }

  // No copy
  LibUsbDevice(const LibUsbDevice &) = delete;
  LibUsbDevice & operator=(const LibUsbDevice &) = delete;

  // Move allowed
  LibUsbDevice(LibUsbDevice && other) noexcept
  : handle_(other.handle_)
  {
    other.handle_ = nullptr;
  }

  LibUsbDevice & operator=(LibUsbDevice && other) noexcept
  {
    if (this != &other) {
      reset();
      handle_ = other.handle_;
      other.handle_ = nullptr;
    }
    return *this;
  }

  void reset()
  {
    if (handle_ != nullptr) {
      libusb_release_interface(handle_, 2);
      libusb_close(handle_);
      handle_ = nullptr;
    }
  }

  libusb_device_handle * get() const
  {
    return handle_;
  }

  libusb_device_handle * release()
  {
    auto h = handle_;
    handle_ = nullptr;
    return h;
  }

  explicit operator bool() const
  {
    return handle_ != nullptr;
  }

private:
  libusb_device_handle * handle_;
};

}  // namespace odrive
