# Error Reference

Complete reference for ODrive error codes exposed through ROS2 state interfaces.

## Error State Interfaces

Each joint exposes these error code state interfaces (as `double`):

- `<joint>/axis_error` - High-level axis errors
- `<joint>/motor_error` - Motor subsystem errors  
- `<joint>/encoder_error` - Encoder subsystem errors
- `<joint>/controller_error` - Position/velocity controller errors

Values are the raw ODrive error bitmask as a double. Multiple bits can be set simultaneously.

---

## Axis Errors

**State Interface**: `<joint>/axis_error`

| Bit | Hex Value | Name | Description | Common Causes | Recovery |
|-----|-----------|------|-------------|---------------|----------|
| 0 | `0x0000000000000001` | `INVALID_STATE` | Axis in invalid state for requested operation | Requesting closed-loop before calibration | Run encoder offset calibration, then retry |
| 1 | `0x0000000000000002` | `DC_BUS_UNDER_VOLTAGE` | DC bus voltage below minimum | Power supply failure, loose connection | Check power supply, connections |
| 2 | `0x0000000000000004` | `DC_BUS_OVER_VOLTAGE` | DC bus voltage above maximum | Regenerative braking with no brake resistor, sudden deceleration | Install brake resistor, tune decel limits |
| 3 | `0x0000000000000008` | `CURRENT_MEASUREMENT_TIMEOUT` | No current sensor readings | DRV fault, hardware failure | Check motor phases, power cycle ODrive |
| 4 | `0x0000000000000010` | `BRAKE_RESISTOR_DISARMED` | Brake resistor armed but not enabled | Configuration mismatch | Set `odrv0.config.enable_brake_resistor = True` |
| 5 | `0x0000000000000020` | `MOTOR_DISARMED` | Motor disarmed unexpectedly | Safety trigger, E-stop | Check why motor was disarmed |
| 6 | `0x0000000000000040` | `MOTOR_FAILED` | Motor subsystem error | See motor_error | Check `<joint>/motor_error` |
| 7 | `0x0000000000000080` | `ENCODER_FAILED` | Encoder subsystem error | See encoder_error | Check `<joint>/encoder_error` |
| 8 | `0x0000000000000100` | `CONTROLLER_FAILED` | Controller subsystem error | See controller_error | Check `<joint>/controller_error` |
| 9 | `0x0000000000000200` | `POS_CTRL_DURING_SENSORLESS` | Position control attempted in sensorless mode | Misconfiguration | Use velocity control or add encoder |
| 10 | `0x0000000000000400` | `WATCHDOG_TIMER_EXPIRED` | No control commands received within timeout | Control loop stopped, USB cable disconnected | **Most common error**: Ensure controller is active, check USB connection |
| 11 | `0x0000000000000800` | `MIN_ENDSTOP_PRESSED` | Minimum endstop triggered | Axis hit mechanical limit | Reverse direction, check endstop wiring |
| 12 | `0x0000000000001000` | `MAX_ENDSTOP_PRESSED` | Maximum endstop triggered | Axis hit mechanical limit | Reverse direction, check endstop wiring |
| 13 | `0x0000000000002000` | `ESTOP_REQUESTED` | Emergency stop commanded | E-stop button, safety system | Investigate trigger, clear E-stop |
| 14 | `0x0000000000004000` | `HOMING_WITHOUT_ENDSTOP` | Homing requested without endstop configured | Misconfiguration | Configure endstop or disable homing |
| 15 | `0x0000000000008000` | `OVER_TEMP` | Temperature limit exceeded | Insufficient cooling, excessive load | Allow cooling, check thermal management |

### Axis Error Priority

When multiple errors occur, diagnose in this order:

1. **`WATCHDOG_TIMER_EXPIRED`** - Indicates control loop issue (see [Troubleshooting: Watchdog Timer Expired](troubleshooting.md#watchdog-timer-expired))
2. **`ENCODER_FAILED`** / **`MOTOR_FAILED`** - Check subsystem errors
3. **Power errors** (`DC_BUS_*`) - Power supply issue
4. **`OVER_TEMP`** - Thermal issue
5. **Others** - Specific to configuration/usage

---

## Motor Errors

**State Interface**: `<joint>/motor_error`

| Bit | Hex Value | Name | Description | Common Causes | Recovery |
|-----|-----------|------|-------------|---------------|----------|
| 0 | `0x0000000000000001` | `PHASE_RESISTANCE_OUT_OF_RANGE` | Measured resistance outside expected range | Incorrect motor config, damaged winding | Verify motor parameters, check winding resistance |
| 1 | `0x0000000000000002` | `PHASE_INDUCTANCE_OUT_OF_RANGE` | Measured inductance outside expected range | Incorrect motor config | Verify motor parameters |
| 2 | `0x0000000000000004` | `ADC_FAILED` | Current ADC malfunction | Hardware failure | Power cycle, replace ODrive if persists |
| 3 | `0x0000000000000008` | `DRV_FAULT` | DRV8301 gate driver fault | Motor short, over-current, over-temp | **Common**: Check motor phase wiring, reduce current limit |
| 4 | `0x0000000000000010` | `CONTROL_DEADLINE_MISSED` | Motor control loop missed deadline | CPU overload, firmware issue | Reduce axis count, update firmware |
| 5 | `0x0000000000000020` | `NOT_IMPLEMENTED_MOTOR_TYPE` | Unsupported motor type selected | Configuration error | Set `motor.config.motor_type` to supported value |
| 6 | `0x0000000000000040` | `BRAKE_CURRENT_OUT_OF_RANGE` | Brake current reading invalid | Hardware issue | Check brake resistor wiring |
| 7 | `0x0000000000000080` | `MODULATION_MAGNITUDE_OUT_OF_RANGE` | PWM modulation out of range | Voltage/current limits misconfigured | Check voltage limits |
| 8 | `0x0000000000000100` | `BRAKE_DEADTIME_VIOLATION` | Brake timing violated | Firmware bug | Update firmware |
| 9 | `0x0000000000000200` | `UNEXPECTED_TIMER_CALLBACK` | Motor control timer anomaly | Firmware bug | Power cycle, update firmware |
| 10 | `0x0000000000000400` | `CURRENT_SENSE_SATURATION` | Current sensor saturated | Over-current condition | Reduce current limit, check for motor short |
| 11 | `0x0000000000000800` | `INVERTER_OVER_TEMP` | MOSFET temperature too high | Insufficient cooling, excessive current | **Common**: Add heatsink, reduce current, improve airflow |
| 13 | `0x0000000000002000` | `CURRENT_UNSTABLE` | Current control unstable | Tuning issue, hardware fault | Re-run motor calibration |
| 14 | `0x0000000000004000` | `DC_BUS_OVER_REGEN_CURRENT` | Excessive regenerative current | No brake resistor, rapid deceleration | Install brake resistor, reduce decel rate |
| 15 | `0x0000000000008000` | `DC_BUS_OVER_CURRENT` | Excessive bus current draw | Short circuit, stall | Check motor wiring, reduce current limit |

### Most Common Motor Errors

1. **`DRV_FAULT`** (bit 3)
   - **Symptom**: Motor doesn't respond, LED flashes
   - **First check**: Motor phase wiring (A, B, C connections)
   - **Second check**: Current limit too high for motor
   - **Solution**: 
     ```python
     odrv0.axis0.motor.config.current_lim = <safe_value>  # e.g., 20A
     odrv0.save_configuration()
     odrv0.reboot()
     ```

2. **`INVERTER_OVER_TEMP`** (bit 11)
   - **Symptom**: Intermittent failures during operation
   - **Check**: ODrive board temperature
   - **Solution**: Improve cooling (heatsink, fan, reduce ambient temp)

---

## Encoder Errors

**State Interface**: `<joint>/encoder_error`

| Bit | Hex Value | Name | Description | Common Causes | Recovery |
|-----|-----------|------|-------------|---------------|----------|
| 0 | `0x0000000000000001` | `UNSTABLE_GAIN` | Encoder gain unstable | Poor signal quality, noise | Check encoder wiring, add shielding |
| 1 | `0x0000000000000002` | `CPR_POLEPAIRS_MISMATCH` | CPR doesn't match motor pole pairs | Configuration error | Set correct `encoder.config.cpr` |
| 2 | `0x0000000000000004` | `NO_RESPONSE` | Encoder not responding | Wiring issue, damaged encoder | Check encoder power (if required), signal wiring |
| 3 | `0x0000000000000008` | `UNSUPPORTED_ENCODER_MODE` | Encoder mode not supported | Configuration error | Use supported encoder type |
| 4 | `0x0000000000000010` | `ILLEGAL_HALL_STATE` | Invalid Hall sensor state | Hall sensor wiring, damaged sensor | Check Hall wiring (often U, V, W order) |
| 5 | `0x0000000000000020` | `INDEX_NOT_FOUND_YET` | Index pulse not detected during calibration | Index wiring, motor not rotating enough | Ensure motor can rotate freely during calibration |
| 6 | `0x0000000000000040` | `ABS_SPI_TIMEOUT` | SPI absolute encoder timeout | SPI wiring, encoder fault | Check SPI connections (MISO, MOSI, SCK, CS) |
| 7 | `0x0000000000000080` | `ABS_SPI_COM_FAIL` | SPI communication failed | SPI wiring, electrical noise | Shorten SPI wires, add pullup resistors |
| 8 | `0x0000000000000100` | `ABS_SPI_NOT_READY` | Absolute encoder not ready | Encoder initialization issue | Power cycle encoder |
| 9 | `0x0000000000000200` | `HALL_NOT_CALIBRATED_YET` | Hall sensors require calibration | Missing calibration | Run `requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION` |

### Encoder Debugging Tips

**For incremental encoders** (A/B/Z):
```python
# Check counts are incrementing
odrv0.axis0.encoder.shadow_count  # Read multiple times while rotating motor
```

**For absolute encoders** (SPI):
```python
# Check encoder responding
odrv0.axis0.encoder.spi_error_rate  # Should be 0.0
```

**For Hall sensors**:
```python
# Check Hall state changes
hex(odrv0.axis0.encoder.hall_state)  # Should cycle through 1-6 as motor rotates
```

---

## Controller Errors

**State Interface**: `<joint>/controller_error`

| Bit | Hex Value | Name | Description | Common Causes | Recovery |
|-----|-----------|------|-------------|---------------|----------|
| 0 | `0x0000000000000001` | `OVERSPEED` | Velocity exceeded limit | Mechanical failure, incorrect limit | Check `controller.config.vel_limit` |
| 1 | `0x0000000000000002` | `INVALID_LOAD_ENCODER` | Load encoder (if used) invalid | Load encoder wiring | Check load encoder configuration |
| 2 | `0x0000000000000004` | `INVALID_ESTIMATE` | Position/velocity estimate invalid | Encoder glitches, extreme acceleration | Check encoder signal quality |

### Controller Error Notes

These errors are less common than axis/motor/encoder errors. Usually indicate:

- **`OVERSPEED`**: Controller tracking error exceeded safety limit (usually means mechanical issue or encoder failure)
- **`INVALID_ESTIMATE`**: Encoder signal degraded or lost

---

## Error Combinations

Some errors occur together:

### `MOTOR_FAILED` + `DRV_FAULT`
Motor driver fault. Check motor phase connections and current limits.

### `ENCODER_FAILED` + `NO_RESPONSE`
Encoder not communicating. Check encoder power and signal wiring.

### `WATCHDOG_TIMER_EXPIRED` (repeated)
Control loop not running consistently:
1. Check controller is active: `ros2 control list_controllers`
2. Check hardware is active: `ros2 control list_hardware_components`
3. Increase watchdog timeout if control loop legitimately slower

### `DC_BUS_OVER_VOLTAGE` + `DC_BUS_OVER_REGEN_CURRENT`
Regenerative braking with no dissipation path. **Solution**: Install brake resistor.

---

## Clearing Errors

### Via Hardware Lifecycle

**Preferred method**:
```bash
# Deactivate hardware (clears errors)
ros2 service call /controller_manager/set_hardware_component_state \
  controller_manager_msgs/srv/SetHardwareComponentState \
  "{name: 'odrive', target_state: {id: 2, label: 'inactive'}}"

# Reactivate
ros2 service call /controller_manager/set_hardware_component_state \
  controller_manager_msgs/srv/SetHardwareComponentState \
  "{name: 'odrive', target_state: {id: 3, label: 'active'}}"
```

This triggers ODrive axis state transition: `CLOSED_LOOP_CONTROL` → `IDLE` → `CLOSED_LOOP_CONTROL`

### Via ODrive CLI (for persistent issues)

```python
import odrive
odrv0 = odrive.find_any()

# Clear errors
odrv0.clear_errors()

# Or dump error for debugging
odrv0.axis0.error  # Should be 0 after clear
odrv0.axis0.motor.error
odrv0.axis0.encoder.error
```

---

## Error Monitoring Strategy

### ROS2 State Interfaces

The `<joint>/healthy` interface combines all error states:

```python
healthy = (axis_error == 0.0) and
          (motor_error == 0.0) and  
          (encoder_error == 0.0) and
          (controller_error == 0.0)
```

**Usage in your code**:
```python
joint_state_msg = self.joint_state_sub.data

for i, name in enumerate(joint_state_msg.name):
    healthy_idx = joint_state_msg.name.index(f"{name}/healthy")
    if joint_state_msg.position[healthy_idx] == 0.0:
        # Joint unhealthy, check error interfaces
        axis_err_idx = joint_state_msg.name.index(f"{name}/axis_error")
        axis_err = int(joint_state_msg.position[axis_err_idx])
        
        if axis_err & 0x400:  # WATCHDOG_TIMER_EXPIRED
            # Handle watchdog expiry
```

### Diagnostics Topic

Higher-level monitoring via `/diagnostics`:

```bash
ros2 topic echo /diagnostics
```

Look for status level:
- `OK` (0): All joints healthy
- `WARN` (1): Non-critical errors (e.g., rate limiting)
- `ERROR` (2): Joint unhealthy
- `STALE` (3): Data not updating

---

## Error Decoding Example

**Scenario**: Joint reports unhealthy

```bash
$ ros2 topic echo /joint_states
name: ['left_wheel', 'left_wheel/healthy', 'left_wheel/axis_error', ...]
position: [3.2, 0.0, 2048.0, ...]
                ^     ^
                |     axis_error = 2048 (decimal)
                healthy = false
```

**Decode**:
```python
# 2048 decimal = 0x800 hex = bit 11
# Bit 11 = WATCHDOG_TIMER_EXPIRED
```

**Solution**: See [Troubleshooting: Watchdog Timer Expired](troubleshooting.md#watchdog-timer-expired)

---

## Error Prevention Checklist

### Before First Power-On

- [ ] Verify motor wiring (phase order matters for Hall sensors)
- [ ] Set appropriate current limits (`motor.config.current_lim`)
- [ ] Configure brake resistor if using regenerative braking
- [ ] Run motor calibration sequence
- [ ] Run encoder offset calibration
- [ ] Set watchdog timeout appropriate for control loop rate

### Before Production Deployment

- [ ] Test E-stop behavior (does watchdog trip correctly?)
- [ ] Verify error recovery (can system recover from transient errors?)
- [ ] Load test (does it run for hours without accumulating errors?)
- [ ] Thermal test (does `INVERTER_OVER_TEMP` occur under sustained load?)
- [ ] Monitor diagnostics for any warnings during normal operation

---

## See Also

- [Troubleshooting Guide](troubleshooting.md) - Solutions to common problems
- [Safety Design](safety-design.md) - How errors fit into safety architecture
- [ODrive Documentation](https://docs.odriverobotics.com/) - Detailed hardware/firmware docs
