# Configuration Guide

Detailed parameter reference and tuning examples for ODrive ROS2 Control.

## Quick Start

Minimal working configuration:

```xml
<ros2_control name="odrive" type="system">
  <hardware>
    <plugin>odrive_hardware_interface/ODriveHardwareInterface</plugin>
    <param name="serial_number">YOUR_SERIAL_NUMBER</param>
  </hardware>
  
  <joint name="my_joint">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    <param name="axis_id">0</param>
  </joint>
</ros2_control>
```

This uses all defaults (see [Default Values](#default-values) section).

---

## Hardware Parameters

These apply to the entire ODrive device (multiple axes).

### Required Parameters

#### `serial_number`
- **Type**: `string` (hex format)
- **Description**: Unique ODrive serial number for USB device selection
- **How to find**:
  ```bash
  odrivetool
  # In odrivetool: hex(odrv0.serial_number)
  # Example output: '0x208037823548'
  ```
- **Example**:
  ```xml
  <param name="serial_number">0x208037823548</param>
  ```

### Optional Parameters

#### `watchdog_timeout`
- **Type**: `double` (seconds)
- **Default**: `0.1` (100ms)
- **Description**: Max time between writes before ODrive enters fault state
- **Tuning**:
  - **Aggressive (fast loop)**: `0.05` (50ms) for 20+ Hz control
  - **Standard**: `0.1` (100ms) for 10 Hz control
  - **Conservative**: `0.2` (200ms) for 5 Hz control or unreliable transport
- **Example**:
  ```xml
  <param name="watchdog_timeout">0.15</param>
  ```
- **See also**: [Safety Design: Watchdog](safety-design.md#1-watchdog-safety)

#### `mask_faulted_axes`
- **Type**: `bool`
- **Default**: `true`
- **Description**: When `true`, faulted axes don't receive commands (prevents watchdog cascade)
- **When to disable**: If you want all axes to fault together (e.g., dual-motor gantry)
- **Example**:
  ```xml
  <param name="mask_faulted_axes">false</param>  <!-- Fault propagation -->
  ```
- **See also**: [Safety Design: Fault Masking](safety-design.md#2-fault-masking)

#### `usb_timeout_ms`
- **Type**: `int`
- **Default**: `100`
- **Description**: Timeout for individual USB read/write operations
- **Tuning**:
  - **Don't change unless necessary** (fixed for real-time determinism)
  - Increase only if persistent transport timeouts on known-good hardware: `150`-`200`
- **Example**:
  ```xml
  <param name="usb_timeout_ms">150</param>
  ```
- **See also**: [Safety Design: Transport Timeout Rationale](safety-design.md#timeout-rationale)

#### `usb_read_retries` / `usb_write_retries`
- **Type**: `int`
- **Default**: `3` (both)
- **Description**: Number of retries for failed USB operations before reporting error
- **Tuning**:
  - **Reliable USB (short cable, low EMI)**: `3` (default)
  - **Challenging environment**: `5`-`7`
  - **Don't exceed ~10** (indicates underlying problem)
- **Example**:
  ```xml
  <param name="usb_read_retries">5</param>
  <param name="usb_write_retries">5</param>
  ```

#### `publish_diagnostics`
- **Type**: `bool`
- **Default**: `true`
- **Description**: Enable `/diagnostics` topic publishing
- **When to disable**: Minimal embedded systems where diagnostics overhead matters
- **Example**:
  ```xml
  <param name="publish_diagnostics">false</param>
  ```

#### `diagnostics_period_sec`
- **Type**: `double` (seconds)
- **Default**: `0.5`
- **Min**: `0.1` (enforced)
- **Description**: Rate of diagnostics publishing
- **Tuning**:
  - **Standard monitoring**: `0.5` (2 Hz)
  - **High-frequency monitoring**: `0.1` (10 Hz) - minimum allowed
  - **Low-priority monitoring**: `1.0` (1 Hz)
- **Example**:
  ```xml
  <param name="diagnostics_period_sec">0.25</param>  <!-- 4 Hz -->
  ```

#### `max_read_cycle_time_sec` / `max_write_cycle_time_sec`
- **Type**: `double` (seconds)
- **Default**: `0.010` (10ms for both)
- **Description**: Threshold for logging deadline miss warnings
- **Tuning**: Set to ~1.5× your expected control loop period
  - **10 Hz control (100ms period)**: `0.150` (150ms threshold)
  - **20 Hz control (50ms period)**: `0.075` (75ms threshold)
  - **50 Hz control (20ms period)**: `0.030` (30ms threshold)
- **Example**:
  ```xml
  <param name="max_read_cycle_time_sec">0.015</param>
  <param name="max_write_cycle_time_sec">0.015</param>
  ```

---

## Joint Parameters

These apply per joint (per axis).

### Required Parameters

#### `axis_id`
- **Type**: `int`
- **Values**: `0` or `1` (ODrive v3.x has 2 axes)
- **Description**: Which ODrive axis this joint corresponds to
- **Example**:
  ```xml
  <joint name="left_wheel">
    <param name="axis_id">0</param>
  </joint>
  <joint name="right_wheel">
    <param name="axis_id">1</param>
  </joint>
  ```

### Optional Parameters

#### Command Validation Limits

These enforce safety bounds on commands from controllers.

##### `command_position_max` / `command_position_min`
- **Type**: `double` (radians)
- **Default**: `±6.28` (±2π, full rotation)
- **Description**: Acceptable range for position commands
- **Tuning**:
  - **Continuous rotation (wheels)**: Use defaults
  - **Limited joint**: Set to mechanical limits
    ```xml
    <param name="command_position_min">-1.57</param>  <!-- -90° -->
    <param name="command_position_max">1.57</param>   <!-- +90° -->
    ```

##### `command_velocity_max`
- **Type**: `double` (rad/s)
- **Default**: `50.0`
- **Description**: Maximum velocity command magnitude
- **Tuning**: Based on motor specs and gearing
  ```python
  # Example: 6000 RPM motor, 10:1 gearing
  max_motor_rpm = 6000
  gear_ratio = 10
  max_output_rps = (max_motor_rpm / gear_ratio) / 60
  max_output_rad_s = max_output_rps * 2 * pi  # ~62.8 rad/s
  ```
  ```xml
  <param name="command_velocity_max">62.8</param>
  ```

##### `command_effort_max`
- **Type**: `double` (Nm)
- **Default**: `10.0`
- **Description**: Maximum torque command magnitude
- **Tuning**: Based on motor torque constant and current limit
  ```python
  # Example: 0.1 Nm/A torque constant, 20A current limit, 10:1 gearing
  torque_constant = 0.1  # Nm/A
  current_limit = 20     # A
  gear_ratio = 10
  max_torque = torque_constant * current_limit * gear_ratio  # 20 Nm
  ```
  ```xml
  <param name="command_effort_max">20.0</param>
  ```

#### Rate Limiting

Protects against sudden command changes that could damage mechanics.

##### `max_position_rate`
- **Type**: `double` (rad/s)
- **Default**: `3.14` (π rad/s, ~0.5 rotations/sec)
- **Description**: Maximum rate of change for position commands
- **Tuning**: Based on mechanical constraints
  ```xml
  <!-- Fast joint: Allow 2 rotations/sec -->
  <param name="max_position_rate">12.56</param>  <!-- 4π -->
  
  <!-- Slow/heavy joint: Conservative rate -->
  <param name="max_position_rate">1.57</param>   <!-- π/2 -->
  ```

##### `max_velocity_rate` / `max_effort_rate`
- **Type**: `double` (rad/s² or Nm/s)
- **Default**: `10.0` (both)
- **Description**: Maximum acceleration/jerk limits
- **Example**:
  ```xml
  <param name="max_velocity_rate">5.0</param>   <!-- Gentle acceleration -->
  <param name="max_effort_rate">20.0</param>     <!-- Allow fast torque changes -->
  ```

#### Feedback Validation

Sanity checks on sensor readings (detect encoder failures).

##### `max_believable_velocity`
- **Type**: `double` (rad/s)
- **Default**: `100.0`
- **Description**: If encoder reports velocity beyond this, mark telemetry invalid
- **Tuning**: Set to ~2× maximum expected velocity
  ```xml
  <param name="max_believable_velocity">125.0</param>
  ```

##### `max_believable_effort`
- **Type**: `double` (Nm)
- **Default**: `50.0`
- **Description**: If current sense reports torque beyond this, mark telemetry invalid
- **Tuning**: Set to ~2× maximum expected torque
  ```xml
  <param name="max_believable_effort">40.0</param>
  ```

---

## Default Values Summary

| Parameter | Default | Unit |
|-----------|---------|------|
| `watchdog_timeout` | `0.1` | seconds |
| `mask_faulted_axes` | `true` | bool |
| `usb_timeout_ms` | `100` | milliseconds |
| `usb_read_retries` | `3` | count |
| `usb_write_retries` | `3` | count |
| `publish_diagnostics` | `true` | bool |
| `diagnostics_period_sec` | `0.5` | seconds |
| `max_read_cycle_time_sec` | `0.010` | seconds |
| `max_write_cycle_time_sec` | `0.010` | seconds |
| `command_position_min` | `-6.28` | radians |
| `command_position_max` | `6.28` | radians |
| `command_velocity_max` | `50.0` | rad/s |
| `command_effort_max` | `10.0` | Nm |
| `max_position_rate` | `3.14` | rad/s |
| `max_velocity_rate` | `10.0` | rad/s² |
| `max_effort_rate` | `10.0` | Nm/s |
| `max_believable_velocity` | `100.0` | rad/s |
| `max_believable_effort` | `50.0` | Nm |

---

## Example Configurations

### Example 1: Differential Drive Robot

**Specs**:
- 2 drive wheels (ODrive axis 0, 1)
- 6000 RPM motors with 0.05 Nm/A torque constant
- 15:1 gearing
- 30A current limit
- 10 Hz control loop

**Configuration**:

```xml
<ros2_control name="odrive_base" type="system">
  <hardware>
    <plugin>odrive_hardware_interface/ODriveHardwareInterface</plugin>
    <param name="serial_number">0x208037823548</param>
    
    <!-- 10 Hz control → 100ms period, watchdog 150ms -->
    <param name="watchdog_timeout">0.15</param>
    
    <!-- Fault together (diff drive should be symmetric) -->
    <param name="mask_faulted_axes">false</param>
    
    <!-- Diagnostics at 2 Hz -->
    <param name="diagnostics_period_sec">0.5</param>
    
    <!-- Deadline threshold: 1.5× control period -->
    <param name="max_read_cycle_time_sec">0.150</param>
    <param name="max_write_cycle_time_sec">0.150</param>
  </hardware>
  
  <joint name="left_wheel">
    <command_interface name="velocity"/>  <!-- Velocity control -->
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    
    <param name="axis_id">0</param>
    
    <!-- Wheel: No position limits (continuous rotation) -->
    <!-- Use defaults: ±6.28 rad -->
    
    <!-- Max velocity: 6000 RPM / 15 gearing = 400 RPM = 41.9 rad/s -->
    <param name="command_velocity_max">42.0</param>
    
    <!-- Max torque: 0.05 Nm/A × 30A × 15 gearing = 22.5 Nm -->
    <param name="command_effort_max">22.5</param>
    
    <!-- Rate limits: Allow fast response but protect mechanics -->
    <param name="max_velocity_rate">20.0</param>  <!-- 20 rad/s² -->
    
    <!-- Feedback validation: 2× max expected -->
    <param name="max_believable_velocity">84.0</param>
    <param name="max_believable_effort">45.0</param>
  </joint>
  
  <joint name="right_wheel">
    <!-- Same as left_wheel -->
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    
    <param name="axis_id">1</param>
    <param name="command_velocity_max">42.0</param>
    <param name="command_effort_max">22.5</param>
    <param name="max_velocity_rate">20.0</param>
    <param name="max_believable_velocity">84.0</param>
    <param name="max_believable_effort">45.0</param>
  </joint>
</ros2_control>
```

### Example 2: Quadruped Leg (Position Control)

**Specs**:
- 1 hip joint (limited range: ±60°)
- High-torque motor: 0.2 Nm/A, 40A limit, 5:1 gearing
- 50 Hz control loop (high-bandwidth)

**Configuration**:

```xml
<ros2_control name="odrive_leg" type="system">
  <hardware>
    <plugin>odrive_hardware_interface/ODriveHardwareInterface</plugin>
    <param name="serial_number">0x369C4F065748</param>
    
    <!-- 50 Hz control → 20ms period, watchdog 30ms -->
    <param name="watchdog_timeout">0.030</param>
    
    <!-- Single joint: mask_faulted_axes doesn't matter -->
    
    <!-- Diagnostics at 10 Hz (max rate) -->
    <param name="diagnostics_period_sec">0.1</param>
    
    <!-- Tight deadline: 1.5× 20ms = 30ms -->
    <param name="max_read_cycle_time_sec">0.030</param>
    <param name="max_write_cycle_time_sec">0.030</param>
  </hardware>
  
  <joint name="hip_joint">
    <command_interface name="position"/>  <!-- Position control -->
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    
    <param name="axis_id">0</param>
    
    <!-- Limited joint: ±60° = ±1.047 rad -->
    <param name="command_position_min">-1.047</param>
    <param name="command_position_max">1.047</param>
    
    <!-- Max velocity: Conservative for position control -->
    <param name="command_velocity_max">10.0</param>  <!-- ~1.6 rev/s -->
    
    <!-- Max torque: 0.2 Nm/A × 40A × 5 gearing = 40 Nm -->
    <param name="command_effort_max">40.0</param>
    
    <!-- Rate limits: Fast but safe -->
    <param name="max_position_rate">6.28</param>   <!-- 1 rev/s -->
    <param name="max_velocity_rate">50.0</param>   <!-- High bandwidth -->
    
    <!-- Feedback validation -->
    <param name="max_believable_velocity">20.0</param>
    <param name="max_believable_effort">80.0</param>
  </joint>
</ros2_control>
```

### Example 3: Industrial Manipulator (Multi-ODrive)

**Specs**:
- 4 joints across 2 ODrives (2 joints each)
- 20 Hz control loop
- Fault independence (one joint fails, others continue)

**Configuration**:

```xml
<!-- First ODrive: Shoulder and Elbow -->
<ros2_control name="odrive_arm_base" type="system">
  <hardware>
    <plugin>odrive_hardware_interface/ODriveHardwareInterface</plugin>
    <param name="serial_number">0xABCD1234ABCD</param>
    <param name="watchdog_timeout">0.075</param>  <!-- 20 Hz → 50ms + margin -->
    <param name="mask_faulted_axes">true</param>  <!-- Independent joints -->
    <param name="diagnostics_period_sec">0.5</param>
    <param name="max_read_cycle_time_sec">0.075</param>
    <param name="max_write_cycle_time_sec">0.075</param>
  </hardware>
  
  <joint name="shoulder_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <param name="axis_id">0</param>
    <param name="command_position_min">-3.14</param>
    <param name="command_position_max">3.14</param>
    <param name="max_position_rate">2.0</param>  <!-- Slow for safety -->
  </joint>
  
  <joint name="elbow_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <param name="axis_id">1</param>
    <param name="command_position_min">0.0</param>
    <param name="command_position_max">2.36</param>  <!-- 135° -->
    <param name="max_position_rate">2.0</param>
  </joint>
</ros2_control>

<!-- Second ODrive: Wrist joints -->
<ros2_control name="odrive_arm_wrist" type="system">
  <hardware>
    <plugin>odrive_hardware_interface/ODriveHardwareInterface</plugin>
    <param name="serial_number">0x1234ABCD5678</param>
    <!-- Same timing as base ODrive -->
    <param name="watchdog_timeout">0.075</param>
    <param name="mask_faulted_axes">true</param>
  </hardware>
  
  <joint name="wrist_pitch">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <param name="axis_id">0</param>
    <param name="command_position_min">-1.57</param>
    <param name="command_position_max">1.57</param>
    <param name="max_position_rate">4.0</param>  <!-- Faster for small joint -->
  </joint>
  
  <joint name="wrist_roll">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <param name="axis_id">1</param>
    <param name="command_position_min">-3.14</param>
    <param name="command_position_max">3.14</param>
    <param name="max_position_rate">6.0</param>  <!-- Continuous rotation OK -->
  </joint>
</ros2_control>
```

### Example 4: Harsh Environment (High EMI, Long Cables)

**Specs**:
- Mobile outdoor robot
- Long USB cables (~3m)
- High electrical noise from inverters
- Needs reliability over performance

**Configuration**:

```xml
<ros2_control name="odrive_outdoor" type="system">
  <hardware>
    <plugin>odrive_hardware_interface/ODriveHardwareInterface</plugin>
    <param name="serial_number">0x5678DCBA9876</param>
    
    <!-- Conservative timing for unreliable transport -->
    <param name="watchdog_timeout">0.25</param>    <!-- 250ms -->
    <param name="usb_timeout_ms">200</param>        <!-- Increased from 100ms -->
    <param name="usb_read_retries">7</param>        <!-- Increased from 3 -->
    <param name="usb_write_retries">7</param>
    
    <!-- Diagnostics: Lower frequency to reduce USB traffic -->
    <param name="diagnostics_period_sec">1.0</param>  <!-- 1 Hz -->
    
    <!-- Lenient deadlines -->
    <param name="max_read_cycle_time_sec">0.250</param>
    <param name="max_write_cycle_time_sec">0.250</param>
  </hardware>
  
  <joint name="wheel">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    
    <param name="axis_id">0</param>
    
    <!-- Conservative rate limits (even if motor could go faster) -->
    <param name="command_velocity_max">30.0</param>
    <param name="max_velocity_rate">5.0</param>  <!-- Gentle acceleration -->
  </joint>
</ros2_control>
```

---

## Tuning Workflow

### Step 1: Start with Defaults

Use minimal configuration (only `serial_number` and `axis_id`):

```xml
<hardware>
  <plugin>odrive_hardware_interface/ODriveHardwareInterface</plugin>
  <param name="serial_number">YOUR_SERIAL</param>
</hardware>
<joint name="test_joint">
  <param name="axis_id">0</param>
  <!-- All other params use defaults -->
</joint>
```

### Step 2: Run and Monitor

```bash
# Activate hardware and controller
ros2 control set_hardware_component_state odrive active
ros2 control set_controller_state <controller> active

# Watch diagnostics
ros2 topic echo /diagnostics
```

Look for:
- ❌ Repeated `WATCHDOG_TIMER_EXPIRED` → Increase `watchdog_timeout`
- ❌ Frequent transport errors → Increase `usb_*_retries`
- ❌ Deadline misses → Increase `max_*_cycle_time_sec`
- ❌ Rate limiting warnings → Increase rate limits or reduce controller aggressiveness

### Step 3: Tune Safety Limits

Check if validation rejects legitimate commands:

```bash
ros2 topic echo /joint_states | grep -E "(position|velocity|effort)_rate_limited"
```

If any show `1.0` (rate limited):
- **Expected during large setpoint changes**: OK
- **Continuous**: Increase corresponding `max_*_rate`

### Step 4: Adjust for Environment

- **Reliable setup (lab, short cables)**: Can use aggressive timing (defaults are conservative)
- **Unreliable setup (mobile, EMI)**: Increase timeouts and retries

### Step 5: Load Testing

Run for extended period (hours) and check diagnostics statistics:

```bash
ros2 topic echo /diagnostics | grep -A10 "stats"
```

Monitor:
- `stats.max_read_time_ms` / `stats.max_write_time_ms` - Should be well below deadline
- `stats.deadline_read_misses` / `stats.deadline_write_misses` - Should be <1% of cycle count
- `stats.usb_read_retries` / `stats.usb_write_retries` - Low retry count indicates good USB reliability

---

## Parameter Interactions

### `watchdog_timeout` ↔ Control Loop Rate

**Rule**: `watchdog_timeout` ≥ 1.5× control loop period

| Control Loop | Period | Recommended Watchdog |
|--------------|--------|----------------------|
| 5 Hz | 200ms | 0.3s |
| 10 Hz | 100ms | 0.15s |
| 20 Hz | 50ms | 0.075s |
| 50 Hz | 20ms | 0.030s |

### `max_position_rate` ↔ `command_velocity_max`

If position control with feedforward velocity:

```
max_position_rate ≤ command_velocity_max
```

Otherwise, position rate limiter conflicts with velocity commands.

### `max_*_cycle_time` ↔ Diagnostics Period

Diagnostics collection adds ~1-2ms overhead. If very tight deadlines:

```xml
<param name="diagnostics_period_sec">1.0</param>  <!-- Reduce frequency -->
```

Or disable entirely:
```xml
<param name="publish_diagnostics">false</param>
```

---

## Advanced: Plugin Integration

If using ODrive hardware interface as a plugin (not via URDF), set configuration programmatically:

```cpp
#include <odrive_hardware_interface/odrive_hardware_interface.hpp>

hardware_interface::HardwareInfo info;
info.name = "odrive";
info.type = "system";

// Hardware parameters
info.hardware_parameters["serial_number"] = "0x208037823548";
info.hardware_parameters["watchdog_timeout"] = "0.1";
info.hardware_parameters["mask_faulted_axes"] = "true";

// Joint info
hardware_interface::ComponentInfo joint_info;
joint_info.name = "wheel";
joint_info.type = "joint";
joint_info.parameters["axis_id"] = "0";
joint_info.parameters["command_velocity_max"] = "50.0";

info.joints.push_back(joint_info);

// Initialize hardware interface
odrive_hardware_interface::ODriveHardwareInterface hw;
hw.on_init(info);
```

---

## See Also

- [Safety Design](safety-design.md) - Rationale for safety parameters
- [Troubleshooting](troubleshooting.md) - Solutions when parameters need adjustment
- [Error Reference](error-reference.md) - Understanding error states
