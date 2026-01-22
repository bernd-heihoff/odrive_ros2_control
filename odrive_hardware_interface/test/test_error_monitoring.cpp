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

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "odrive_hardware_interface/error_monitoring.hpp"

#include "rcutils/error_handling.h"
#include "rcutils/logging.h"

namespace
{
constexpr char kLoggerName[] = "ODriveHardwareInterface";

std::mutex g_log_mutex;
std::vector<std::string> g_log_messages;

void capture_log(
  const rcutils_log_location_t *,
  int,
  const char *,
  rcutils_time_point_value_t,
  const char * format,
  va_list * args)
{
  char buffer[1024];
  va_list args_copy;
  va_copy(args_copy, *args);
  vsnprintf(buffer, sizeof(buffer), format, args_copy);
  va_end(args_copy);

  std::lock_guard<std::mutex> lock(g_log_mutex);
  g_log_messages.emplace_back(buffer);
}

class ScopedLogCapture
{
public:
  ScopedLogCapture()
  : previous_handler_(rcutils_logging_get_output_handler())
  {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    g_log_messages.clear();
    rcutils_logging_set_output_handler(capture_log);
  }

  ~ScopedLogCapture()
  {
    rcutils_logging_set_output_handler(previous_handler_);
  }

  static std::vector<std::string> logs()
  {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    return g_log_messages;
  }

  static void clear()
  {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    g_log_messages.clear();
  }

private:
  rcutils_logging_output_handler_t previous_handler_;
};
}  // namespace

namespace odrive_hardware_interface
{
namespace
{
TEST(ErrorMonitoringTest, ExtractErrorValueHandlesEdgeCases)
{
  EXPECT_FALSE(extract_error_value(std::numeric_limits<double>::quiet_NaN()).has_value());
  EXPECT_FALSE(extract_error_value(-1.0).has_value());
  EXPECT_FALSE(extract_error_value(0.0).has_value());

  auto positive = extract_error_value(42.0);
  ASSERT_TRUE(positive.has_value());
  EXPECT_EQ(42u, positive.value());

  auto truncated = extract_error_value(5.75);
  ASSERT_TRUE(truncated.has_value());
  EXPECT_EQ(5u, truncated.value());
}

TEST(ErrorMonitoringTest, AxisErrorTransitionLogsDecodedBits)
{
  ScopedLogCapture capture;
  ASSERT_EQ(
    RCUTILS_RET_OK,
    rcutils_logging_set_logger_level(kLoggerName, RCUTILS_LOG_SEVERITY_WARN));

  std::uint64_t last = 0;
  log_axis_error_transition("steering_joint", 0x0000000000000041ULL, last);

  const auto logs = ScopedLogCapture::logs();
  ASSERT_EQ(1u, logs.size());
  EXPECT_NE(std::string::npos, logs[0].find("steering_joint"));
  EXPECT_NE(std::string::npos, logs[0].find("0x0000000000000041"));
  EXPECT_NE(std::string::npos, logs[0].find("invalid_state"));
  EXPECT_NE(std::string::npos, logs[0].find("motor_failed"));
  EXPECT_EQ(0x0000000000000041ULL, last);

  ScopedLogCapture::clear();
  log_axis_error_transition("steering_joint", 0x0000000000000041ULL, last);
  EXPECT_TRUE(ScopedLogCapture::logs().empty());
}

}  // namespace
}  // namespace odrive_hardware_interface
