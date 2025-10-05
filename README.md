# odrive_ros2_control

`odrive_ros2_control` provides a ROS 2 hardware interface and reference demos for controlling [ODrive](https://odriverobotics.com) brushless motor controllers through the `ros2_control` framework. The hardware interface speaks the native ODrive USB protocol, exposes torque/velocity/position command surfaces, and reports rich telemetry for monitoring and recovery.

This repository is maintained for the Wackerbot platform and regularly rebased on the upstream Factor Robotics implementation. The `wackerbot` branch targets ROS 2 Humble Hawksbill.

## Package summary

- `odrive_hardware_interface` – the `hardware_interface::SystemInterface` plugin, configuration parser, command validation utilities, and transport adapters.
- `odrive_demo_description` – xacro/URDF fragments that model an ODrive and pre-populate `ros2_control` parameters.
- `odrive_demo_bringup` – launch files and controller configurations demonstrating common control modes.
- `odrive_ros2_control` – meta-package that wires the above components together for discovery and installation.

## Compatibility

| ROS 2 | ODrive firmware | Branch |
| --- | --- | --- |
| Humble Hawksbill | v0.5.1 | [wackerbot](../../tree/wackerbot) |

> **Tip:** Ensure ROS 2 Humble (desktop or base) and `libusb-1.0` headers are installed before building.

## Features

- Native USB transport with watchdog integration and recovery support.
- Position, velocity, and torque command modes with seamless mode switching.
- Comprehensive state feedback (position, velocity, effort, temperatures, and axis/motor/encoder/controller error bitmasks).
- Parameter-driven configuration for multiple ODrives and axes within one hardware instance.
- Command-limit validation to guard against NAN / out-of-range set points before they reach the drive.
- Example URDF and launch files that plug directly into `ros2_control` demo controllers.
- Extensive unit tests that exercise configuration parsing, safety checks, telemetry conversion, and error monitoring helpers.

## Requirements

- ROS 2 Humble Hawksbill.
- `libusb-1.0` runtime and development headers.
- ODrive firmware v0.5.1 configured for USB control.
- A `colcon` workspace with the `ros-humble-ros2-control` stack installed.

## Getting started

```bash
# 1. Create or reuse a ROS 2 workspace.
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Clone the repository (or add as a submodule).
git clone https://github.com/bernd-heihoff/wackerbot_humble_ws.git

# 3. Install dependencies (libusb and rosdep keys for controllers / demos).
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y

# 4. Build and source the overlay.
colcon build --packages-select odrive_ros2_control odrive_hardware_interface \
	odrive_demo_bringup odrive_demo_description
source install/setup.bash
```

> The repository includes additional packages that are not strictly required for ODrive; select only the ones you need in the `colcon build` call to keep cycle times short.

## Example bring-up

The demo packages provide launch files to spin up simulated robots against an ODrive (or the mock transport used in the tests):

```bash
# Diff-drive base using simulated controllers
ros2 launch odrive_demo_bringup odrive_diffbot.launch.py

# Two-axis RR manipulator example
ros2 launch odrive_demo_bringup odrive_rrbot.launch.py

# Demonstration of mixed command interfaces across axes
ros2 launch odrive_demo_bringup odrive_multi_interface.launch.py
```

Each launch file loads the URDF from `odrive_demo_description`, starts the `controller_manager`, and spawns the configured controllers. Swap in your own robot description or controller YAML to match the hardware you intend to drive.

## Hardware configuration

The hardware interface consumes the `ros2_control` hardware block defined in URDF/Xacro. The `odrive_demo_description/urdf/odrive.ros2_control.xacro` macro is the recommended starting point:

```xml
<ros2_control name="odrive" type="system">
	<hardware>
		<plugin>odrive_hardware_interface/ODriveHardwareInterface</plugin>
	</hardware>

	<sensor name="odrv0">
		<param name="serial_number">206735905548</param>
	</sensor>

	<joint name="left_wheel">
		<param name="serial_number">206735905548</param>
		<param name="axis">0</param>
		<param name="enable_watchdog">true</param>
		<param name="watchdog_timeout">0.1</param>
		<param name="command_position_min">-6.28</param>
		<param name="command_position_max">6.28</param>
		<param name="enforce_command_limits">true</param>
	</joint>
	<!-- repeat joint block for axis 1 -->
</ros2_control>
```

Key parameters per joint:

| Parameter | Type | Required | Description |
| --- | --- | --- | --- |
| `serial_number` | hex / integer | ✅ | Serial of the ODrive that hosts the axis (hex strings are accepted and parsed case-insensitively). |
| `axis` | integer (0 or 1) | ✅ | Axis index on the selected ODrive. |
| `enable_watchdog` | bool | ✅ | Enables drive watchdog feeding during command streaming; set to `false` for development rigs without watchdog. |
| `watchdog_timeout` | seconds (double) | ✅ | Timeout programmed into the drive when `enable_watchdog` is true. |
| `command_*_min/max` | double | optional | Optional safety limits for position (rad), velocity (rad/s), and effort (Nm); the interface rejects commands that violate enforced limits. |
| `enforce_command_limits` | bool | optional | Defaults to `true`; set to `false` to log limit violations without blocking commands. |

Add a `<sensor>` entry per unique ODrive you monitor for bus voltage. When multiple ODrives share the same serial, include one `<sensor>` and multiple `<joint>` entries pointing to the same serial with different axes.

## Diagnostics and monitoring

- Axis / motor / encoder / controller error transitions are logged via `rclcpp::Logger` warnings with decoded bit masks. Use the new unit tests (see below) as a reference for expected log messages.
- The hardware interface resets telemetry to `NaN` when transport communication drops, making it easy to detect stale data in controllers.
- `recover()` can be called through lifecycle transitions to reinitialize the transport, clear errors, and resume operation without restarting the process.

## Testing

```bash
colcon test --packages-select odrive_hardware_interface
colcon test-result --verbose
```

The suite includes mock-transport gtests that cover:

- Safe mode switching and command dispatch (`test_axis_control`).
- Telemetry conversions and error reads (`test_axis_telemetry`).
- Hardware configuration parsing edge cases (`test_odrive_configuration`).
- Command limit enforcement and watchdog interactions (`test_command_safety`).
- Lifecycle recovery and transport reinitialisation (`test_lifecycle`).
- Error monitoring log transitions and `extract_error_value` helpers (`test_error_monitoring`).

Source your ROS installation (e.g. `source /opt/ros/humble/setup.bash`) before running tests so that controller dependencies and shared libraries are discoverable.

## Roadmap

- [ ] Support for serial and CAN transports.
- [ ] Auto-configuration from URDF/YAML manifests.
- [ ] Expanded diagnostics publishers for bus voltage and temperature trends.

## Further reading

- [Project wiki](https://github.com/Factor-Robotics/odrive_ros2_control/wiki/Documentation) – upstream documentation, firmware notes, and calibration tips.
- [ros2_control documentation](https://control.ros.org/) – controller manager usage, interface concepts, and best practices.
- [`odrive_demo_description` URDF](odrive_demo_description/urdf/odrive.ros2_control.xacro) – ready-to-use macro for your robot description.

## License

Licensed under the Apache License, Version 2.0. See the [LICENSE](LICENSE) file for details.
