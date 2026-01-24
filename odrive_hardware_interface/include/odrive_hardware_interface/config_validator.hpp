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

#include <sstream>
#include <string>

#include "rcutils/logging_macros.h"

namespace odrive_hardware_interface
{

/// Helper for validating and parsing configuration parameters with consistent logging.
class ConfigValidator
{
public:
  /// Validate that a numeric value falls within an acceptable range.
  ///
  /// @param logger_name Logger name for warning messages
  /// @param param_name Human-readable parameter name for error messages
  /// @param value Value to validate
  /// @param min_value Minimum acceptable value (inclusive)
  /// @param max_value Maximum acceptable value (inclusive)
  /// @param default_value Fallback value if validation fails
  /// @param[out] out_value Validated value (either input or default)
  /// @return true if value was in range, false if default was used
  template<typename T>
  static bool validate_range(
    const char * logger_name,
    const char * param_name,
    T value,
    T min_value,
    T max_value,
    T default_value,
    T & out_value)
  {
    if (value >= min_value && value <= max_value) {
      out_value = value;
      return true;
    }

    std::ostringstream min_str, max_str, val_str, def_str;
    min_str << min_value;
    max_str << max_value;
    val_str << value;
    def_str << default_value;

    RCUTILS_LOG_WARN_NAMED(
      logger_name,
      "%s value %s out of range [%s, %s]; using default %s",
      param_name,
      val_str.str().c_str(),
      min_str.str().c_str(),
      max_str.str().c_str(),
      def_str.str().c_str());

    out_value = default_value;
    return false;
  }

  /// Validate and parse an unsigned integer parameter from string.
  ///
  /// @param logger_name Logger name for warning messages
  /// @param param_name Parameter name for error messages
  /// @param value_str String value to parse
  /// @param min_value Minimum acceptable value
  /// @param max_value Maximum acceptable value
  /// @param default_value Fallback value if parsing or validation fails
  /// @param[out] out_value Validated value
  /// @return true if parsing and validation succeeded
  static bool parse_unsigned_int(
    const char * logger_name,
    const char * param_name,
    const std::string & value_str,
    unsigned int min_value,
    unsigned int max_value,
    unsigned int default_value,
    unsigned int & out_value)
  {
    try {
      unsigned int parsed = std::stoul(value_str);
      return validate_range(
        logger_name, param_name, parsed, min_value, max_value,
        default_value, out_value);
    } catch (const std::exception &) {
      RCUTILS_LOG_WARN_NAMED(
        logger_name,
        "Invalid %s value '%s'; using default %u",
        param_name, value_str.c_str(), default_value);
      out_value = default_value;
      return false;
    }
  }

  /// Validate and parse a double parameter from string.
  ///
  /// @param logger_name Logger name for warning messages
  /// @param param_name Parameter name for error messages
  /// @param value_str String value to parse
  /// @param min_value Minimum acceptable value
  /// @param max_value Maximum acceptable value
  /// @param default_value Fallback value if parsing or validation fails
  /// @param[out] out_value Validated value
  /// @return true if parsing and validation succeeded
  static bool parse_double(
    const char * logger_name,
    const char * param_name,
    const std::string & value_str,
    double min_value,
    double max_value,
    double default_value,
    double & out_value)
  {
    try {
      std::size_t processed = 0;
      double parsed = std::stod(value_str, &processed);
      if (processed != value_str.size()) {
        throw std::invalid_argument("incomplete parse");
      }
      return validate_range(
        logger_name, param_name, parsed, min_value, max_value,
        default_value, out_value);
    } catch (const std::exception &) {
      RCUTILS_LOG_WARN_NAMED(
        logger_name,
        "Invalid %s value '%s'; using default %f",
        param_name, value_str.c_str(), default_value);
      out_value = default_value;
      return false;
    }
  }
};

}  // namespace odrive_hardware_interface
