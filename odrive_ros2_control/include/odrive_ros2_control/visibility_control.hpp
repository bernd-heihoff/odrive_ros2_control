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

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ODRIVE_ROS2_CONTROL_EXPORT __attribute__ ((dllexport))
    #define ODRIVE_ROS2_CONTROL_IMPORT __attribute__ ((dllimport))
  #else
    #define ODRIVE_ROS2_CONTROL_EXPORT __declspec(dllexport)
    #define ODRIVE_ROS2_CONTROL_IMPORT __declspec(dllimport)
  #endif
  #ifdef ODRIVE_ROS2_CONTROL_BUILDING_DLL
    #define ODRIVE_ROS2_CONTROL_PUBLIC ODRIVE_ROS2_CONTROL_EXPORT
  #else
    #define ODRIVE_ROS2_CONTROL_PUBLIC ODRIVE_ROS2_CONTROL_IMPORT
  #endif
  #define ODRIVE_ROS2_CONTROL_PUBLIC_TYPE ODRIVE_ROS2_CONTROL_PUBLIC
  #define ODRIVE_ROS2_CONTROL_LOCAL
#else
  #define ODRIVE_ROS2_CONTROL_EXPORT __attribute__ ((visibility("default")))
  #define ODRIVE_ROS2_CONTROL_IMPORT
  #if __GNUC__ >= 4
    #define ODRIVE_ROS2_CONTROL_PUBLIC __attribute__ ((visibility("default")))
    #define ODRIVE_ROS2_CONTROL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ODRIVE_ROS2_CONTROL_PUBLIC
    #define ODRIVE_ROS2_CONTROL_LOCAL
  #endif
  #define ODRIVE_ROS2_CONTROL_PUBLIC_TYPE
#endif
