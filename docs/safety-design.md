# Safety Design

This document describes the safety architecture of the ODrive ROS2 Control hardware interface and explains the design rationale behind key safety features.

## Overview

The ODrive hardware interface implements multiple layers of safety to protect against:
- Control loop failures
- Hardware faults (motor, encoder, axis errors)
- Communication failures (USB disconnect, timeouts)
- Invalid commands (NaN, out-of-range, excessive rates)
- Lifecycle state violations

**Design Philosophy**: Fail fast and fail safe. The system prioritizes surfacing problems immediately over attempting to "work around" failures, enabling proper fault handling at the application level.

## Watchdog Safety

### Design

The ODrive firmware includes a hardware watchdog timer that must be "fed" periodically. **Critically**, this implementation feeds the watchdog ONLY during `write()` operations when commands are actively being sent.

```cpp
// Watchdog feed occurs ONLY in write() - NOT in read()
if (joint.enable_watchdog) {
  transport->call(serial, AXIS__WATCHDOG_FEED);
}
```

### Rationale

This is **intentional** and differs from traditional "heartbeat" designs:

- **Control loop failure detection**: If the controller stops commanding (crashed, blocked, etc.), the watchdog expires and the motor stops
- **Lifecycle gating**: When hardware is INACTIVE, commands don't flow → watchdog expires → motors safely stop
- **Fault masking**: When an axis is masked due to faults, no commands → watchdog expires → motor stops

**Key Point**: The watchdog detects control failures, not just communication failures. Set `watchdog_timeout` conservatively above your worst-case control loop period.

### Configuration

```xml
<joint name="wheel">
  <param name="enable_watchdog">true</param>
  <param name="watchdog_timeout">0.1</param>  <!-- 100ms - must exceed control loop period -->
</joint>
```

**Recommendations**:
- Differential drive robots: `0.05 - 0.1s` (50-100ms)
- Manipulators: `0.1 - 0.2s` (100-200ms)
- Slow-moving platforms: `0.2 - 0.5s` (200-500ms)

⚠️ **Warning**: Setting too short can cause false triggers during normal operation. Setting too long reduces safety response time.

## Fault Masking

### Feature: `mask_faulted_axes`

When enabled (default: `true`), faulted axes are automatically masked - commands are skipped while healthy axes continue operating.

```yaml
hardware_parameters:
  mask_faulted_axes: true  # default
```

### Behavior

```
Cycle 1: [Axis 0: Healthy] [Axis 1: Healthy]     → Both receive commands
Cycle 2: [Axis 0: Healthy] [Axis 1: FAULT!]      → Only Axis 0 receives commands
Cycle 3: [Axis 0: Healthy] [Axis 1: FAULT]       → Only Axis 0 receives commands
Cycle 4: [Axis 0: Healthy] [Axis 1: Recovered]   → Both receive commands
```

### Use Cases

**Differential Drive** (RECOMMENDED: `true`):
- One wheel motor faults → robot can still drive straight (degraded mode)
- Enables "limp home" capability
- Prevents total immobilization from single-point failure

**Synchronized Systems** (CONSIDER: `false`):
- Multi-axis arms where one failed axis makes motion unsafe
- Gantry systems requiring coordinated motion
- Any application where partial operation is more dangerous than full stop

### Interaction with `request_idle_on_axis_fault`

```yaml
hardware_parameters:
  mask_faulted_axes: true
  request_idle_on_axis_fault: true  # default
```

When both enabled:
1. Fault detected on Axis 1
2. Axis 1 transitioned to IDLE (motor disabled)
3. Axis 1 commands masked (skipped)
4. Axis 0 continues normal operation

This prevents faulted motors from continuing to try to operate.

## Lifecycle Gating

### Feature: `gate_outputs_with_lifecycle`

When enabled (default: `true`), motor commands are ONLY sent when the hardware component is in ACTIVE state.

```yaml
hardware_parameters:
  gate_outputs_with_lifecycle: true  # default
```

### State Transitions

```
┌─────────────┐
│ UNCONFIGURED│  No hardware access
└──────┬──────┘
       │ on_init()
┌──────▼──────┐
│   INACTIVE  │  Hardware configured but not connected
└──────┬──────┘  Commands BLOCKED
       │ on_activate()
       │  - Connect to USB
       │  - Read motor parameters
       │  - Clear errors
       │  - Enable outputs
┌──────▼──────┐
│    ACTIVE   │  Hardware connected and operational
└──────┬──────┘  Commands FLOWING
       │ on_deactivate()
       │  - Send IDLE to all axes
       │  - Disable outputs
┌──────▼──────┐
│   INACTIVE  │  Commands BLOCKED
└─────────────┘
```

### Safety Benefits

1. **Controlled startup**: Motors don't move until explicitly activated
2. **Clean shutdown**: `on_deactivate()` transitions all motors to IDLE before blocking commands
3. **Integration safety**: Controllers can't accidentally command unprepared hardware
4. **Emergency stop integration**: External systems can trigger deactivate

### Write Behavior by State

| State | `write()` Behavior |
|-------|-------------------|
| UNCONFIGURED | Returns OK (no-op) |
| INACTIVE | Returns OK (commands blocked) |
| ACTIVE | Commands sent to hardware |

**Note**: `read()` always proceeds regardless of state (telemetry always available).

## Command Validation

### Multi-Layer Validation

Commands pass through multiple validation stages before reaching hardware:

```
Controller Command
    ↓
1. NaN/Infinity Check
    ↓
2. Range Validation (min/max limits)
    ↓
3. Rate Limiting (max change per cycle)
    ↓
4. Control Mode Validation
    ↓
5. USB Protocol Conversion
    ↓
Hardware
```

### 1. NaN/Infinity Protection

```cpp
if (!std::isfinite(command_position)) {
  // Reject command, fall back to zero torque
  validation_failures++;
}
```

**Why**: NaN values can propagate from crashed nodes, failed computations, or uninitialized memory. Prevents undefined motor behavior.

### 2. Range Validation

```xml
<joint name="wheel">
  <param name="command_position_min">-6.28</param>  <!-- -2π rad -->
  <param name="command_position_max">6.28</param>   <!-- +2π rad -->
  <param name="command_velocity_min">-10.0</param>
  <param name="command_velocity_max">10.0</param>
  <param name="command_effort_min">-5.0</param>     <!-- ±5 Nm -->
  <param name="command_effort_max">5.0</param>
</joint>
```

Commands exceeding limits are **rejected** with error logging. The system falls back to holding current position with zero feed-forward torque.

**Design Choice**: Hard rejection vs clamping. We reject to surface controller bugs rather than silently modifying commands.

### 3. Rate Limiting

```xml
<joint name="wheel">
  <param name="max_position_rate">3.14</param>    <!-- π rad/s max -->
  <param name="max_velocity_rate">5.0</param>     <!-- 5 rad/s² max -->
  <param name="max_effort_rate">10.0</param>      <!-- 10 Nm/s max -->
</joint>
```

**Purpose**: Prevents mechanical shock, protects gearboxes, improves safety around humans.

Rate limiting is **applied** (command modified) with warning logs, not rejected. State interface `<joint>/position_rate_limited` indicates when active.

### 4. Feedback Validation

Input sanity checking on sensor readings:

```xml
<joint name="wheel">
  <param name="max_believable_velocity">50.0</param>        <!-- 50 rad/s -->
  <param name="max_believable_effort">20.0</param>          <!-- 20 Nm -->
  <param name="max_position_discontinuity">1.57</param>     <!-- π/2 rad -->
</joint>
```

If feedback exceeds believable limits:
- Reading marked invalid (`telemetry_valid = 0`)
- Joint marked unhealthy (`healthy = 0`)
- Last valid value retained
- Warning logged

**Why**: Detects encoder glitches, EMI-induced corruption, firmware bugs.

## Transport Error Handling

### Fixed Timeout Strategy

**All timeouts are intentionally FIXED** (not adaptive) for deterministic real-time behavior:

```yaml
hardware_parameters:
  usb_timeout_ms: 100           # Max wait per USB operation
  usb_read_retries: 3           # Retry attempts
  usb_write_retries: 3
  usb_retry_delay_ms: 5         # Fixed delay between retries
```

### Rationale

1. **Deterministic timing**: Control loop knows worst-case execution time
2. **Fail fast**: Bad hardware surfaces immediately, not hidden by increasing retries
3. **Predictable debugging**: Consistent behavior across all failure modes
4. **Real-time guarantees**: Cannot allow indefinite blocking

**Worst-case time**: `usb_timeout_ms + (usb_retries × usb_retry_delay_ms)`  
Example: `100 + (3 × 5) = 115ms` per operation

### Transport Failure Response

| Failure Type | System Response |
|--------------|-----------------|
| USB disconnect | All joints marked `unhealthy`, telemetry invalid, commands skipped |
| Timeout on read | Joint-specific: that joint marked `unhealthy`, others continue |
| Timeout on write | Joint-specific: `write_error` set, logged, other joints continue |
| Persistent failures | Accumulate in diagnostics, trigger warnings |

**Recovery**: Requires `on_deactivate()` → `on_activate()` cycle to reinitialize transport.

## Error State Monitoring

### Health Flag: `<joint>/healthy`

**Single source of truth** for axis health, computed fresh every cycle:

```cpp
bool errors_clear = 
  (axis_error == 0.0) &&
  (motor_error == 0.0) &&
  (encoder_error == 0.0) &&
  (controller_error == 0.0);

joint.healthy = errors_clear ? 1.0 : 0.0;
```

**Use this for application-level safety decisions**, not the individual error registers.

### Error Registers

Available as state interfaces:
- `<joint>/axis_error` - Axis-level errors (invalid_state, watchdog, over_temp, etc.)
- `<joint>/motor_error` - Motor-level errors (DRV fault, over-current, phase issues, etc.)
- `<joint>/encoder_error` - Encoder errors (calibration, communication, index issues)
- `<joint>/controller_error` - Controller errors (overspeed, instability, invalid config)

### Error Transition Logging

Errors are logged **only when they change** (not every cycle):

```
[WARN] Axis error for joint 'wheel' changed from 0x0000000000000000 to 0x0000000000000001 (invalid_state)
```

This prevents log spam while ensuring you're notified of all error transitions.

## Real-Time Deadline Monitoring

### Cycle Time Monitoring

```yaml
hardware_parameters:
  max_read_cycle_time_sec: 0.010    # 10ms deadline
  max_write_cycle_time_sec: 0.010   # 10ms deadline
  enable_deadline_warnings: true
```

**Purpose**: Detect when I/O operations exceed expected time, indicating:
- Too many axes on single USB bus
- USB interference/EMI
- System overload
- Real-time scheduling issues

### Statistics Available

Published in diagnostics:
- `stats.read_cycles` - Total read operations
- `stats.write_cycles` - Total write operations
- `stats.read_deadline_misses` - Count of slow reads
- `stats.write_deadline_misses` - Count of slow writes
- `stats.max_read_time_ms` - Worst-case read time observed
- `stats.max_write_time_ms` - Worst-case write time observed

**Interpretation**:
- Occasional misses (<1%): Normal, likely due to USB bus contention
- Frequent misses (>5%): Investigate system load, USB bandwidth, or reduce axis count
- Consistent misses (>20%): Real-time kernel may be needed, or hardware insufficient

## Safety Checklist

Before deploying in production:

- [ ] Watchdog enabled and timeout tuned for your control loop period
- [ ] Command limits configured for your mechanical range of motion
- [ ] Rate limits set to prevent mechanical damage
- [ ] Feedback limits set to catch sensor glitches
- [ ] `mask_faulted_axes` configured appropriately for your application
- [ ] Lifecycle gating enabled unless you have specific reasons to disable
- [ ] USB retry configuration tested in your EMI environment
- [ ] Deadline monitoring enabled and thresholds validated
- [ ] Error recovery procedures documented
- [ ] Diagnostics monitoring configured for fleet management
- [ ] Emergency stop tested and integrated with lifecycle transitions

## Integration with External Safety Systems

This hardware interface provides **motor-level safety**. For complete system safety, integrate with:

1. **Safety Supervisor Node** (`wackerbot_safety_supervisor`)
   - Monitors `<joint>/healthy` flags
   - Enforces velocity limits based on environment
   - Triggers lifecycle transitions on faults

2. **Hardware E-Stop**
   - Should deactivate hardware interface (trigger `on_deactivate()`)
   - Independent of software control

3. **Navigation Stack Safety**
   - Collision avoidance
   - Obstacle detection
   - Path validity

4. **Battery Management**
   - Monitor `<sensor>/vbus_voltage`
   - Trigger safe shutdown on low battery

## References

- [Configuration Guide](configuration-guide.md) - Detailed parameter tuning
- [Error Reference](error-reference.md) - Complete error code listings
- [Troubleshooting](troubleshooting.md) - Common issues and solutions
