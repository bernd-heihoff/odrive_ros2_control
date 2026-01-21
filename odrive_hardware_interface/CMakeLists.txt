cmake_minimum_required(VERSION 3.5)

project(odrive_hardware_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-implicit-fallthrough)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include(FindPkgConfig)
pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)

if(LIBUSB1_FOUND)
  include_directories(SYSTEM ${LIBUSB1_INCLUDE_DIRS})
else()
  message( FATAL_ERROR "Failed to find libusb-1.0" )
endif()

ament_auto_add_library(
  odrive_usb SHARED
  src/odrive_usb.cpp
)
target_link_libraries(
  odrive_usb
  ${LIBUSB1_LIBRARIES}
)
ament_target_dependencies(odrive_usb rcutils)

ament_auto_add_library(
  odrive_core SHARED
  src/axis_control.cpp
  src/axis_telemetry.cpp
  src/command_validation.cpp
  src/error_monitoring.cpp
)
ament_target_dependencies(odrive_core rcutils)

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/odrive_hardware_interface.cpp
  src/odrive_configuration.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  odrive_core
)

ament_target_dependencies(${PROJECT_NAME} std_srvs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_FILE_EXCLUDE
    "include/odrive_hardware_interface/odrive_endpoints.hpp"
    "include/odrive_hardware_interface/install/_local_setup_util_ps1.py"
    "include/odrive_hardware_interface/install/_local_setup_util_sh.py")
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # NOTE: In this environment ament_copyright crashes with:
  # AttributeError: 'function' object has no attribute 'file_headers'
  # This disables the linter until the underlying toolchain issue is resolved.
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  set(TEST_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test)

  ament_add_gtest(test_axis_control test/test_axis_control.cpp)
  if(TARGET test_axis_control)
    target_include_directories(test_axis_control PRIVATE ${TEST_INCLUDE_DIR})
    target_link_libraries(test_axis_control odrive_core)
  endif()

  ament_add_gtest(test_axis_telemetry test/test_axis_telemetry.cpp)
  if(TARGET test_axis_telemetry)
    target_include_directories(test_axis_telemetry PRIVATE ${TEST_INCLUDE_DIR})
    target_link_libraries(test_axis_telemetry odrive_core)
  endif()

  ament_add_gtest(test_odrive_configuration test/test_odrive_configuration.cpp)
  if(TARGET test_odrive_configuration)
    target_include_directories(test_odrive_configuration PRIVATE ${TEST_INCLUDE_DIR})
    target_link_libraries(test_odrive_configuration ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_command_safety test/test_command_safety.cpp)
  if(TARGET test_command_safety)
    target_include_directories(test_command_safety PRIVATE ${TEST_INCLUDE_DIR})
    target_link_libraries(test_command_safety ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_lifecycle test/test_lifecycle.cpp)
  if(TARGET test_lifecycle)
    target_include_directories(test_lifecycle PRIVATE ${TEST_INCLUDE_DIR})
    target_link_libraries(test_lifecycle ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_error_monitoring test/test_error_monitoring.cpp)
  if(TARGET test_error_monitoring)
    target_include_directories(test_error_monitoring PRIVATE ${TEST_INCLUDE_DIR})
    target_link_libraries(test_error_monitoring odrive_core)
  endif()

  ament_add_gtest(test_diagnostics test/test_diagnostics.cpp)
  if(TARGET test_diagnostics)
    target_include_directories(test_diagnostics PRIVATE ${TEST_INCLUDE_DIR})
    target_link_libraries(test_diagnostics ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_safety_gating test/test_safety_gating.cpp)
  if(TARGET test_safety_gating)
    target_include_directories(test_safety_gating PRIVATE ${TEST_INCLUDE_DIR})
    target_link_libraries(test_safety_gating ${PROJECT_NAME})
  endif()
endif()

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
    PATTERN "*.hpp"
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
