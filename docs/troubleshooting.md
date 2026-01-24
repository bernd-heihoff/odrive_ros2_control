# Troubleshooting Guide

Common problems and solutions for ODrive ROS2 Control.

## Quick Diagnostics

Before diving into specific issues, check these basics:

```bash
# 1. Check hardware is detected
ros2 control list_hardware_components

# 2. Check state interfaces are publishing
ros2 topic echo /joint_states

# 3. Check for errors in logs
ros2 run rqt_console rqt_console

# 4. Check diagnostics
ros2 topic echo /diagnostics
```

---

## Activation Failures

### "Device not found" / "Failed to initialize ODrive transport"

**Symptoms**:
```
[ERROR] Transport error (-19) while initialising ODrive transport
[ERROR] Failed to initialize ODrive transport during activation
```

**Cause**: USB device not accessible

**Solutions**:

1. **Check USB connection**:
   ```bash
   lsusb | grep ODrive
   # Should show: Bus XXX Device XXX: ID 1209:0d32 ODrive Robotics ODrive
   ```

2. **Fix USB permissions**:
   ```bash
   # Create udev rule
   echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d32", MODE="0666"' | \
     sudo tee /etc/udev/rules.d/50-odrive.rules
   
   # Reload udev rules
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   
   # Reconnect ODrive or reboot
   ```

3. **Verify serial number matches configuration**:
   ```bash
   # Get actual serial number
   odrivetool
   # In odrivetool: hex(odrv0.serial_number)
   
   # Compare with your URDF/YAML configuration
   ```

4. **Check for device conflicts**:
   ```bash
   # See if another process is using the device
   sudo lsof | grep odrive
   ```

---

## Watchdog Timer Expired

### Symptoms

```
[WARN] Axis error for joint 'wheel' changed to 0x0000000000000800 (watchdog_timer_expired)
```

**Joint becomes unhealthy, motor stops**

### Causes & Solutions

#### 1. Control Loop Not Running

**Check controller is active**:
```bash
ros2 control list_controllers
# Should show: <controller_name> active
```

**Solution**: Start your controller:
```bash
ros2 control load_controller <controller_name>
ros2 control set_controller_state <controller_name> active
```

#### 2. Control Loop Too Slow

**Check cycle times in diagnostics**:
```bash
ros2 topic echo /diagnostics | grep -A5 "write_time"
# Look for stats.max_write_time_ms
```

**Solution**: Increase watchdog timeout:
```xml
<param name="watchdog_timeout">0.2</param>  <!-- was 0.1 -->
```

#### 3. Hardware Inactive

Watchdog feeds only occur when hardware is ACTIVE:

```bash
# Check hardware state
ros2 control list_hardware_components

# Activate if needed
ros2 service call /controller_manager/set_hardware_component_state \
  controller_manager_msgs/srv/SetHardwareComponentState \
  "{name: 'odrive', target_state: {id: 3, label: 'active'}}"
```

#### 4. Axis Faulted and Masked

When `mask_faulted_axes: true`, faulted axes don't receive commands → watchdog expires

**This is expected behavior**. Check why axis faulted:
```bash
ros2 topic echo /joint_states | grep -A10 <joint_name>
# Look at axis_error, motor_error fields
```

---

## Transport Timeouts

### "Transport error (-110) while reading/writing"

**Symptoms**:
```
[WARN] Transport error (-110) reading axis telemetry for joint 'wheel'; continuing
```

Error code -110 = `ETIMEDOUT`

### Causes & Solutions

#### 1. USB Cable Quality

**Try**:
- Use shorter USB cable (<2m preferred)
- Use shielded cable
- Avoid USB hubs
- Try different USB port (USB 3.0 sometimes more reliable than 2.0)

#### 2. Electromagnetic Interference

**Common sources**:
- Motor power cables too close to USB cable
- Switching power supplies
- WiFi modules
- High-current power wiring

**Solutions**:
- Route USB cable away from power cables
- Use shielded/ferrite-beaded USB cable
- Ground motor chassis properly
- Increase retries:
  ```yaml
  hardware_parameters:
    usb_read_retries: 5  # was 3
    usb_write_retries: 5
  ```

#### 3. System Load

**Check CPU usage**:
```bash
top
# Look for high CPU usage, especially on core running control loop
```

**Solutions**:
- Reduce other process load
- Use real-time kernel (`PREEMPT_RT`)
- Increase timeout slightly:
  ```yaml
  hardware_parameters:
    usb_timeout_ms: 150  # was 100
  ```

#### 4. Too Many Axes on One Bus

USB bandwidth is finite. Rule of thumb: **≤8 axes per USB controller**

**Check**:
```bash
lsusb -t  # See USB topology
```

**Solution**: Distribute ODrives across multiple USB controllers

---

## Command Validation Failures

### "Rejected command for joint: <reason>"

**Symptoms**:
```
[ERROR] Rejected command for joint 'wheel': Position command 7.5 exceeds maximum 6.28 (falling back to zero torque)
```

### Solutions

#### 1. Check Controller Output

```bash
# Echo command topic to see what controller is sending
ros2 topic echo /<controller>/commands
```

#### 2. Adjust Command Limits

If controller legitimately needs larger range:
```xml
<param name="command_position_max">10.0</param>  <!-- Increase limit -->
```

#### 3. Fix Controller Bugs

NaN commands usually indicate controller math errors:
```bash
# Enable controller debug logging
ros2 run <controller_package> <controller_node> --ros-args --log-level debug
```

#### 4. Check for Uninitialized Commands

Some controllers publish NaN before receiving first state:
```cpp
// In controller: Initialize command values
command_.position = state_.position;  // Don't leave as NaN
```

---

## Rate Limiting Warnings

### "Rate limited joint '<name>': Position/Velocity/Effort rate limited"

**Symptoms**:
```
[WARN] Rate limited joint 'wheel': Position rate limited from 5.2 to 3.14 (max_rate=3.14 dt=0.01)
```

State interface `<joint>/position_rate_limited` = 1.0

### When This is OK

- **Startup**: Controller ramps to setpoint → brief rate limiting expected
- **Large setpoint changes**: Intentional protection from mechanical shock

### When This is a Problem

- **Continuous rate limiting**: Controller cannot achieve desired performance
- **High-frequency oscillation**: Controller tuning issue

### Solutions

#### 1. Increase Rate Limits (if mechanically safe)

```xml
<param name="max_position_rate">5.0</param>  <!-- was 3.14 -->
<param name="max_velocity_rate">10.0</param>
```

#### 2. Tune Controller Aggressiveness

Reduce controller gains to produce smoother commands:
```yaml
# Example for diff_drive_controller
<controller_name>:
  ros__parameters:
    cmd_vel_timeout: 0.5
    publish_rate: 50.0
    # Reduce acceleration limits
    linear:
      x:
        max_acceleration: 0.5  # was 1.0
```

#### 3. Monitor Continuously

```bash
# Watch rate limiting state
ros2 topic echo /joint_states | grep rate_limited
```

---

## Joint Reports Unhealthy

### Check: `<joint>/healthy` = 0.0

**Meaning**: One or more error registers non-zero

### Diagnosis Steps

1. **Check which error**:
   ```bash
   ros2 topic echo /joint_states
   # Look at axis_error, motor_error, encoder_error, controller_error
   ```

2. **Decode error** (see [error-reference.md](error-reference.md)):
   ```python
   # Example: axis_error = 1.0 (stored as float)
   # 0x0000000000000001 = invalid_state
   ```

3. **Clear errors** (if transient):
   ```bash
   # Deactivate and reactivate hardware
   ros2 service call /controller_manager/set_hardware_component_state \
     controller_manager_msgs/srv/SetHardwareComponentState \
     "{name: 'odrive', target_state: {id: 2, label: 'inactive'}}"
   
   ros2 service call /controller_manager/set_hardware_component_state \
     controller_manager_msgs/srv/SetHardwareComponentState \
     "{name: 'odrive', target_state: {id: 3, label: 'active'}}"
   ```

### Common Error Recovery

| Error | Recovery |
|-------|----------|
| `watchdog_timer_expired` | Ensure control loop running, increase timeout |
| `encoder_failed` | Check encoder wiring, recalibrate |
| `motor_failed` → `drv_fault` | Check motor phases, power supply voltage |
| `over_temp` | Allow cooling, check thermal management |
| `invalid_state` | Axis in wrong state, reactivate hardware |

---

## Telemetry Appears Invalid

### Check: `<joint>/telemetry_valid` = 0.0

**Causes**:

1. **Transport error** during read
2. **Feedback validation failed** (value exceeded believable limit)

### Diagnosis

```bash
# Check read_error
ros2 topic echo /joint_states | grep read_error
# Non-zero = transport issue

# Check feedback limits in logs
ros2 run rqt_console rqt_console
# Look for "Feedback validation failed"
```

### Solutions

#### Transport Errors
See [Transport Timeouts](#transport-timeouts) section

#### Feedback Validation
```bash
# Check if limits too tight
ros2 topic echo /joint_states | grep -E "(velocity|effort)"
# Compare values to configured limits
```

Adjust limits if legitimate values exceeded:
```xml
<param name="max_believable_velocity">100.0</param>  <!-- was 50.0 -->
```

---

## Deadline Warnings

### "read()/write() cycle time exceeded limit"

**Symptoms**:
```
[WARN] read() cycle time 12.5ms exceeded limit 10.0ms (miss 5 of 1000 cycles)
```

### Acceptable Miss Rate

- **<1%**: Normal, due to USB bus contention
- **1-5%**: Monitor, may indicate system loading
- **>5%**: Action required

### Solutions

#### 1. Increase Deadline Threshold

If actual times are consistent but slightly over:
```yaml
hardware_parameters:
  max_read_cycle_time_sec: 0.015  # was 0.010 (10ms → 15ms)
```

#### 2. Reduce Axis Count per Bus

Too many axes saturate USB bandwidth. Split across controllers:
```bash
# Check USB topology
lsusb -t
```

#### 3. Use Real-Time Kernel

Significant improvement for control loop timing:
```bash
# Check if RT kernel installed
uname -a | grep PREEMPT_RT

# If not, install rt kernel package for your distro
```

#### 4. CPU Affinity

Pin control loop to dedicated CPU core:
```bash
# In your launch file
<node pkg="controller_manager" exec="ros2_control_node"
      launch-prefix="taskset -c 2">  <!-- Pin to CPU 2 -->
```

---

## Diagnostics Not Publishing

### Check: No `/diagnostics` topic

**Causes**:

1. **Diagnostics disabled** (default: enabled)
2. **No diagnostics factory** set (plugin configuration issue)

### Solutions

1. **Enable in configuration**:
   ```yaml
   hardware_parameters:
     publish_diagnostics: true
   ```

2. **Check diagnostics appear after activation**:
   ```bash
   ros2 topic hz /diagnostics
   # Should show ~2 Hz (default period: 0.5s)
   ```

3. **For plugin users**: Ensure diagnostics factory is set in plugin initialization

---

## Motor Doesn't Move

### Systematic Check

1. **Hardware state**:
   ```bash
   ros2 control list_hardware_components
   # Should be "active"
   ```

2. **Controller state**:
   ```bash
   ros2 control list_controllers
   # Your controller should be "active"
   ```

3. **Commands flowing**:
   ```bash
   ros2 topic echo /<controller>/commands
   # Should show non-zero values when joystick/cmd_vel sent
   ```

4. **Joint healthy**:
   ```bash
   ros2 topic echo /joint_states | grep healthy
   # Should be 1.0
   ```

5. **ODrive axis state** (via odrivetool):
   ```python
   hex(odrv0.axis0.current_state)
   # Should be 0x8 (CLOSED_LOOP_CONTROL) when active
   ```

6. **Write errors**:
   ```bash
   ros2 topic echo /joint_states | grep write_error
   # Should be 0.0
   ```

### If Commands Not Reaching Hardware

Check `write_error` in joint state. Non-zero indicates USB transport issue.

### If ODrive Not in Closed Loop

Hardware may not be activating correctly. Check activation logs:
```bash
ros2 run rqt_console rqt_console
# Filter for "Activating ODrive hardware"
```

---

## Performance Degradation Over Time

### Symptoms

- Increasing deadline misses
- Occasional transport timeouts
- Memory usage growing

### Solutions

1. **Check for memory leaks**:
   ```bash
   # Monitor process memory
   ps aux | grep ros2_control_node
   ```

2. **Check USB retry accumulation**:
   ```bash
   ros2 topic echo /diagnostics | grep retry
   # Look for stats.usb_read_retries, stats.usb_write_retries
   ```
   
   Growing retry counts indicate degrading USB reliability

3. **Thermal issues**:
   ```bash
   ros2 topic echo /diagnostics | grep temperature
   ```

4. **Restart node periodically** (if truly necessary):
   ```bash
   # In systemd or supervisord config, enable restart policy
   ```

---

## Getting Help

If you've tried the above and still have issues:

1. **Collect diagnostics**:
   ```bash
   ros2 topic echo /diagnostics > diagnostics.log
   ros2 topic echo /joint_states > joint_states.log
   ros2 control list_hardware_components > hw_components.txt
   ```

2. **Check versions**:
   ```bash
   ros2 pkg xml odrive_hardware_interface | grep version
   # ODrive firmware version (via odrivetool): odrv0.fw_version_string
   ```

3. **Enable debug logging**:
   ```bash
   ros2 run <your_launch_pkg> <launch_file> \
     --ros-args --log-level odrive_hardware_interface:=debug
   ```

4. **File an issue** with:
   - System info (ROS version, OS, kernel)
   - Hardware setup (how many ODrives, axes, USB topology)
   - Configuration files (URDF, YAML)
   - Log excerpts showing the problem
   - Diagnostic output

---

## See Also

- [Safety Design](safety-design.md) - Understanding safety features
- [Error Reference](error-reference.md) - Complete error code meanings
- [Configuration Guide](configuration-guide.md) - Parameter tuning
