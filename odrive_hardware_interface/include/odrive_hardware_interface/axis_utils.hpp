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
inline std::int16_t axis_endpoint(std::int16_t base, int axis)
{
  return static_cast<std::int16_t>(base + odrive::per_axis_offset * axis);
}
}  // namespace odrive_hardware_interface
