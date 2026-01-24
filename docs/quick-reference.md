# Quick Reference Card

Common commands and checks for ODrive ROS2 Control operations.

## Health Checks

### Check Hardware Status
```bash
ros2 control list_hardware_components
# Expected: odrive [active]
```

### Check Controller Status
```bash
ros2 control list_controllers
# Expected: <controller_name> [active]
```

### Monitor Joint Health
```bash
ros2 topic echo /joint_states | grep -A3 "healthy"
# healthy = 1.0 (good), 0.0 (unhealthy)
```

### View Diagnostics
```bash
ros2 topic echo /diagnostics
# Check status.level: 0=OK, 1=WARN, 2=ERROR, 3=STALE
```

## Activation & Deactivation

### Activate Hardware
```bash
ros2 service call /controller_manager/set_hardware_component_state \
  controller_manager_msgs/srv/SetHardwareComponentState \
  "{name: 'odrive', target_state: {id: 3, label: 'active'}}"
```

### Deactivate Hardware (Clear Errors)
```bash
ros2 service call /controller_manager/set_hardware_component_state \
  controller_manager_msgs/srv/SetHardwareComponentState \
  "{name: 'odrive', target_state: {id: 2, label: 'inactive'}}"
```

### Activate Controller
```bash
ros2 control load_controller <controller_name>
ros2 control set_controller_state <controller_name> active
```

### Deactivate Controller
```bash
ros2 control set_controller_state <controller_name> inactive
```

## Error Diagnosis

### Check Error Codes
```bash
ros2 topic echo /joint_states | grep -E "(axis_error|motor_error|encoder_error)"
```

### Common Error Decoding

| Error Value | Meaning | Quick Fix |
|-------------|---------|-----------|
| axis_error = 2048.0 | Watchdog expired (0x800) | Start controller, check control loop |
| axis_error = 128.0 | Encoder failed (0x80) | Check encoder wiring |
| motor_error = 8.0 | DRV fault (0x08) | Check motor phases, reduce current |
| motor_error = 2048.0 | Inverter over temp (0x800) | Improve cooling |

**Full reference**: [Error Reference](error-reference.md)

## USB Troubleshooting

### Check USB Device
```bash
lsusb | grep ODrive
# Should show: ID 1209:0d32 ODrive Robotics ODrive
```

### Fix Permissions
```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d32", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/50-odrive.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
# Reconnect ODrive or reboot
```

### Get Serial Number
```bash
odrivetool
# In odrivetool console:
hex(odrv0.serial_number)
# Example output: '0x208037823548'
```

## Performance Monitoring

### Check Cycle Times
```bash
ros2 topic echo /diagnostics | grep -E "(read_time|write_time)"
# stats.avg_read_time_ms, stats.max_read_time_ms, etc.
```

### Monitor Deadline Misses
```bash
ros2 topic echo /diagnostics | grep deadline
# stats.deadline_read_misses, stats.deadline_write_misses
# Should be <1% of total cycles
```

### Check Rate Limiting
```bash
ros2 topic echo /joint_states | grep rate_limited
# 1.0 = actively rate limiting, 0.0 = not limited
```

## Configuration Snippets

### Minimal Configuration
```xml
<ros2_control name="odrive" type="system">
  <hardware>
    <plugin>odrive_hardware_interface/ODriveHardwareInterface</plugin>
    <param name="serial_number">0xYOUR_SERIAL_HERE</param>
  </hardware>
  <joint name="my_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <param name="axis_id">0</param>
  </joint>
</ros2_control>
```

### Increase Watchdog Timeout
```xml
<!-- For 5Hz control loop -->
<param name="watchdog_timeout">0.3</param>  <!-- 300ms -->
```

### Increase USB Retries (Harsh Environment)
```xml
<param name="usb_read_retries">7</param>
<param name="usb_write_retries">7</param>
```

### Adjust Rate Limits
```xml
<joint name="wheel">
  <!-- ... -->
  <param name="max_velocity_rate">20.0</param>  <!-- rad/s² -->
  <param name="max_position_rate">6.28</param>  <!-- rad/s -->
</joint>
```

## Logging

### Enable Debug Logging
```bash
ros2 run <pkg> <node> --ros-args --log-level odrive_hardware_interface:=debug
```

### Filter Logs in rqt_console
```bash
ros2 run rqt_console rqt_console
# Filter by node: /controller_manager
# Filter by severity: WARN, ERROR
```

### Save Diagnostics to File
```bash
ros2 topic echo /diagnostics > diagnostics_$(date +%Y%m%d_%H%M%S).log
```

## ODrive CLI Commands

### Connect to ODrive
```bash
odrivetool
# Auto-connects to first ODrive found
```

### Check Axis State
```python
# In odrivetool:
hex(odrv0.axis0.current_state)
# 0x1 = IDLE
# 0x8 = CLOSED_LOOP_CONTROL (expected when active)
```

### Clear Errors
```python
odrv0.clear_errors()
odrv0.axis0.error  # Should be 0
```

### Check Encoder
```python
# Incremental encoder
odrv0.axis0.encoder.shadow_count  # Should increment when motor rotates

# Absolute encoder
odrv0.axis0.encoder.spi_error_rate  # Should be 0.0
```

### Check Motor Current
```python
odrv0.axis0.motor.current_control.Iq_measured
# Should be non-zero when motor under load
```

## Testing

### Run All Tests
```bash
cd ~/ros2_ws
colcon test --packages-select odrive_hardware_interface
colcon test-result --verbose
```

### Run Specific Test
```bash
colcon test --packages-select odrive_hardware_interface \
  --ctest-args -R test_command_validation
colcon test-result --verbose
```

### Check Test Coverage
```bash
# After running tests
cat ~/ros2_ws/build/odrive_hardware_interface/coverage.txt
```

## State Interface Reference

Each joint exposes these state interfaces:

| Interface | Type | Description |
|-----------|------|-------------|
| `position` | double | Encoder position (rad) |
| `velocity` | double | Encoder velocity (rad/s) |
| `effort` | double | Motor torque (Nm) |
| `healthy` | double | 1.0 = all errors zero, 0.0 = unhealthy |
| `axis_error` | double | Axis error bitmask (as double) |
| `motor_error` | double | Motor error bitmask |
| `encoder_error` | double | Encoder error bitmask |
| `controller_error` | double | Controller error bitmask |
| `iq_setpoint` | double | Commanded q-axis current (A) |
| `iq_measured` | double | Measured q-axis current (A) |
| `temperature` | double | FET temperature (°C) |
| `telemetry_valid` | double | 1.0 = read succeeded, 0.0 = transport error |
| `read_error` | double | USB read error code (0 = success) |
| `write_error` | double | USB write error code (0 = success) |
| `position_rate_limited` | double | 1.0 = rate limited, 0.0 = not limited |
| `velocity_rate_limited` | double | 1.0 = rate limited, 0.0 = not limited |
| `effort_rate_limited` | double | 1.0 = rate limited, 0.0 = not limited |

**Tip**: Use `ros2 topic echo /joint_states` to see all interfaces for all joints.

## Command Interface Reference

Each joint supports these command interfaces:

| Interface | Type | Description |
|-----------|------|-------------|
| `position` | double | Target position (rad) |
| `velocity` | double | Target velocity (rad/s) |
| `effort` | double | Target torque (Nm) |

**Note**: ODrive automatically selects control mode based on last non-zero command.

## Safety Checklist

Before deploying in production:

- [ ] Watchdog timeout set appropriately for control loop rate
- [ ] Command limits configured for mechanical constraints
- [ ] Rate limits tuned to prevent mechanical shock
- [ ] USB permissions configured (no sudo required)
- [ ] Tested error recovery (deactivate → reactivate)
- [ ] Monitored diagnostics during sustained operation
- [ ] Verified thermal management (no OVER_TEMP errors)
- [ ] Load tested for hours without accumulating errors
- [ ] E-stop integration tested (watchdog expires correctly)

**Full checklist**: [Safety Design](safety-design.md#safety-checklist)

## Common Workflows

### Initial Setup
1. Find serial: `odrivetool` → `hex(odrv0.serial_number)`
2. Add to URDF: `<param name="serial_number">0xYOUR_SERIAL</param>`
3. Set permissions: (see USB Troubleshooting above)
4. Build: `colcon build --packages-select odrive_hardware_interface`
5. Source: `source install/setup.bash`

### First Activation
1. Launch: `ros2 launch <your_pkg> <your_launch.py>`
2. Check hardware: `ros2 control list_hardware_components`
3. Activate: (see Activation commands above)
4. Check health: `ros2 topic echo /joint_states | grep healthy`
5. Monitor diagnostics: `ros2 topic echo /diagnostics`

### Debugging Failures
1. Check hardware state: `ros2 control list_hardware_components`
2. Check error codes: `ros2 topic echo /joint_states | grep error`
3. Decode errors: See [Error Reference](error-reference.md)
4. Check logs: `ros2 run rqt_console rqt_console`
5. Follow troubleshooting: [Troubleshooting Guide](troubleshooting.md)

### Parameter Tuning
1. Start with defaults (minimal config)
2. Monitor diagnostics for warnings
3. Adjust one parameter at a time
4. Test thoroughly after each change
5. Document final configuration

**Full tuning guide**: [Configuration Guide](configuration-guide.md#tuning-workflow)

## See Also

- [Safety Design](safety-design.md) - Understand safety architecture
- [Troubleshooting](troubleshooting.md) - Solve specific problems
- [Error Reference](error-reference.md) - Decode error codes
- [Configuration Guide](configuration-guide.md) - Tune parameters
