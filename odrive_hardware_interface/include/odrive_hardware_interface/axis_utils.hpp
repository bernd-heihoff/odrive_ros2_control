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

#include "odrive_hardware_interface/odrive_endpoints.hpp"

namespace odrive_hardware_interface
{
namespace detail
{
constexpr double kPi = 3.141592653589793238462643383279502884;
}  // namespace detail

constexpr double kRadiansToTurns = 1.0 / (2.0 * detail::kPi);
constexpr double kTurnsToRadians = 2.0 * detail::kPi;

inline std::int16_t axis_endpoint(std::int16_t base, int axis)
{
  return static_cast<std::int16_t>(base + odrive::per_axis_offset * axis);
}

template<typename T>
inline T radians_to_turns(T value)
{
  return static_cast<T>(value * static_cast<T>(kRadiansToTurns));
}

template<typename T>
inline T turns_to_radians(T value)
{
  return static_cast<T>(value * static_cast<T>(kTurnsToRadians));
}

}  // namespace odrive_hardware_interface
