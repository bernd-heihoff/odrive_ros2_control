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

#include <gtest/gtest.h>

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <deque>
#include <vector>

#include "odrive_hardware_interface/odrive_transport.hpp"

namespace odrive_hardware_interface
{
// Minimal mock transport capturing read / write / call operations.
class MockTransport : public ODriveTransport
{
public:
  struct ReadExpectation
  {
    std::int64_t serial;
    std::int16_t endpoint;
    std::vector<std::uint8_t> payload;
    int result;
  };

  struct WriteExpectation
  {
    std::int64_t serial;
    std::int16_t endpoint;
    std::vector<std::uint8_t> payload;
    int result;
  };

  struct CallExpectation
  {
    std::int64_t serial;
    std::int16_t endpoint;
    int result;
  };

  int initialize(const SerialMatrix &) override {return 0;}

  template<typename T>
  void expect_read(std::int64_t serial, std::int16_t endpoint, const T & value, int result = 0)
  {
    ReadExpectation expectation;
    expectation.serial = serial;
    expectation.endpoint = endpoint;
    expectation.payload.resize(sizeof(T));
    std::memcpy(expectation.payload.data(), &value, sizeof(T));
    expectation.result = result;
    read_expectations_.push_back(expectation);
  }

  template<typename T>
  void expect_write(std::int64_t serial, std::int16_t endpoint, const T & value, int result = 0)
  {
    WriteExpectation expectation;
    expectation.serial = serial;
    expectation.endpoint = endpoint;
    expectation.payload.resize(sizeof(T));
    std::memcpy(expectation.payload.data(), &value, sizeof(T));
    expectation.result = result;
    write_expectations_.push_back(expectation);
  }

  void expect_call(std::int64_t serial, std::int16_t endpoint, int result = 0)
  {
    call_expectations_.push_back(CallExpectation{serial, endpoint, result});
  }

  bool expectations_satisfied() const
  {
    return read_expectations_.empty() && write_expectations_.empty() && call_expectations_.empty();
  }

  std::size_t pending_writes() const {return write_expectations_.size();}

protected:
  int read_impl(
    std::int64_t serial_number, std::int16_t endpoint_id, void * value,
    std::size_t size) override
  {
    if (read_expectations_.empty()) {
      ADD_FAILURE() << "Unexpected read for endpoint " << endpoint_id;
      return -1;
    }
    auto expectation = read_expectations_.front();
    read_expectations_.pop_front();
    EXPECT_EQ(expectation.serial, serial_number);
    EXPECT_EQ(expectation.endpoint, endpoint_id);
    if (expectation.result != 0) {
      return expectation.result;
    }
    EXPECT_EQ(expectation.payload.size(), size);
    std::memcpy(value, expectation.payload.data(), size);
    return 0;
  }

  int write_impl(
    std::int64_t serial_number, std::int16_t endpoint_id, const void * value,
    std::size_t size) override
  {
    if (write_expectations_.empty()) {
      ADD_FAILURE() << "Unexpected write for endpoint " << endpoint_id;
      return -1;
    }
    auto expectation = write_expectations_.front();
    write_expectations_.pop_front();
    EXPECT_EQ(expectation.serial, serial_number);
    EXPECT_EQ(expectation.endpoint, endpoint_id);
    if (expectation.payload.size() != size) {
      ADD_FAILURE() << "Write size mismatch";
    } else {
      std::vector<std::uint8_t> actual(size);
      std::memcpy(actual.data(), value, size);
      EXPECT_EQ(expectation.payload, actual);
    }
    return expectation.result;
  }

  int call(std::int64_t serial_number, std::int16_t endpoint_id) override
  {
    if (call_expectations_.empty()) {
      ADD_FAILURE() << "Unexpected call for endpoint " << endpoint_id;
      return -1;
    }
    auto expectation = call_expectations_.front();
    call_expectations_.pop_front();
    EXPECT_EQ(expectation.serial, serial_number);
    EXPECT_EQ(expectation.endpoint, endpoint_id);
    return expectation.result;
  }

private:
  std::deque<ReadExpectation> read_expectations_;
  std::deque<WriteExpectation> write_expectations_;
  std::deque<CallExpectation> call_expectations_;
};
}  // namespace odrive_hardware_interface
