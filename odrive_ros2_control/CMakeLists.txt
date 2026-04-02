cmake_minimum_required(VERSION 3.5)
project(odrive_ros2_control)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(
	odrive_ros2_control_ros_diagnostics SHARED
	src/ros_diagnostics_factory.cpp
	src/ros_odrive_hardware_interface_plugin.cpp
)

target_compile_definitions(
	odrive_ros2_control_ros_diagnostics
	PRIVATE ODRIVE_ROS2_CONTROL_BUILDING_DLL)

ament_target_dependencies(
	odrive_ros2_control_ros_diagnostics
	diagnostic_updater
	odrive_hardware_interface
	pluginlib
	rclcpp
)

target_include_directories(
	odrive_ros2_control_ros_diagnostics
	PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

install(
	TARGETS odrive_ros2_control_ros_diagnostics
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

install(
	DIRECTORY include/
	DESTINATION include
)

pluginlib_export_plugin_description_file(hardware_interface odrive_hardware_interface.xml)

ament_export_dependencies(
	diagnostic_updater
	odrive_hardware_interface
	pluginlib
	rclcpp
)
ament_export_include_directories(include)
ament_export_libraries(odrive_ros2_control_ros_diagnostics)

ament_package()
