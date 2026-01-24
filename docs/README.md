# ODrive ROS2 Control Documentation

Complete documentation for the `odrive_ros2_control` package.

## Getting Started

**New to this package?** Start here:

1. **[README](../README.md)** - Package overview, installation, basic usage
2. **[Quick Reference](quick-reference.md)** - Common commands and workflows
3. **[Configuration Guide](configuration-guide.md)** - Set up your robot

## Documentation by Role

### For Operators / End Users

Running and monitoring ODrive-powered robots:

- **[Quick Reference](quick-reference.md)** - Commands at a glance
- **[Troubleshooting Guide](troubleshooting.md)** - Fix common issues
- **[Error Reference](error-reference.md)** - Understand error messages

### For System Integrators

Configuring ODrive for your robot platform:

- **[Configuration Guide](configuration-guide.md)** - Parameter tuning with examples
  - Differential drive robots
  - Quadrupeds  
  - Industrial manipulators
  - Harsh environments
- **[Safety Design](safety-design.md)** - Safety features and rationale
- **[Troubleshooting Guide](troubleshooting.md)** - Debug integration issues

### For Safety Engineers

Validating system safety for production deployment:

- **[Safety Design](safety-design.md)** - Complete safety architecture
  - Watchdog safety (feed-on-write strategy)
  - Fault masking and isolation
  - Command validation and rate limiting
  - Transport error handling
  - Real-time deadline monitoring
- **Safety Checklist** in [Safety Design](safety-design.md#safety-checklist)
- **Error Recovery** in [Error Reference](error-reference.md#clearing-errors)

### For Developers

Contributing to or extending the package:

- **[README](../README.md#testing)** - Run unit tests
- **[Safety Design](safety-design.md)** - Understand design decisions
- **Architecture Diagram** in [README](../README.md#architecture)
- Source code documentation (in-line comments)

## Documentation by Task

### Installation & Setup
- [README: Getting Started](../README.md#getting-started)
- [README: Requirements](../README.md#requirements)
- [Quick Reference: Initial Setup](quick-reference.md#initial-setup)
- [Troubleshooting: USB Permissions](troubleshooting.md#device-not-found--failed-to-initialize-odrive-transport)

### Configuration
- [Configuration Guide: Quick Start](configuration-guide.md#quick-start)
- [Configuration Guide: Hardware Parameters](configuration-guide.md#hardware-parameters)
- [Configuration Guide: Joint Parameters](configuration-guide.md#joint-parameters)
- [Configuration Guide: Example Configurations](configuration-guide.md#example-configurations)
- [Quick Reference: Configuration Snippets](quick-reference.md#configuration-snippets)

### Activation & Lifecycle
- [Quick Reference: Activation & Deactivation](quick-reference.md#activation--deactivation)
- [Quick Reference: First Activation](quick-reference.md#first-activation)
- [Safety Design: Lifecycle Gating](safety-design.md#3-lifecycle-gating)

### Monitoring
- [Quick Reference: Health Checks](quick-reference.md#health-checks)
- [Quick Reference: Performance Monitoring](quick-reference.md#performance-monitoring)
- [Quick Reference: State Interface Reference](quick-reference.md#state-interface-reference)
- [README: Diagnostics and Monitoring](../README.md#diagnostics-and-monitoring)

### Troubleshooting
- [Troubleshooting Guide](troubleshooting.md) - Comprehensive problem-solving
  - Activation failures
  - Watchdog timer expired
  - Transport timeouts
  - Command validation failures
  - Rate limiting
  - Joint reports unhealthy
  - Motor doesn't move
- [Quick Reference: Error Diagnosis](quick-reference.md#error-diagnosis)
- [Quick Reference: Debugging Failures](quick-reference.md#debugging-failures)

### Error Handling
- [Error Reference: Error Codes](error-reference.md) - Complete tables
  - Axis errors
  - Motor errors  
  - Encoder errors
  - Controller errors
- [Error Reference: Error Combinations](error-reference.md#error-combinations)
- [Error Reference: Clearing Errors](error-reference.md#clearing-errors)
- [Quick Reference: Error Diagnosis](quick-reference.md#error-diagnosis)

### Safety Features
- [Safety Design: Watchdog Safety](safety-design.md#1-watchdog-safety)
- [Safety Design: Fault Masking](safety-design.md#2-fault-masking)
- [Safety Design: Lifecycle Gating](safety-design.md#3-lifecycle-gating)
- [Safety Design: Command Validation](safety-design.md#4-command-validation)
- [Safety Design: Transport Error Handling](safety-design.md#5-transport-error-handling)
- [Safety Design: Error State Monitoring](safety-design.md#6-error-state-monitoring)
- [Safety Design: Real-Time Deadline Monitoring](safety-design.md#7-real-time-deadline-monitoring)

### Performance Tuning
- [Configuration Guide: Tuning Workflow](configuration-guide.md#tuning-workflow)
- [Configuration Guide: Parameter Interactions](configuration-guide.md#parameter-interactions)
- [Quick Reference: Performance Monitoring](quick-reference.md#performance-monitoring)
- [Troubleshooting: Deadline Warnings](troubleshooting.md#deadline-warnings)

## Document Status

| Document | Status | Last Updated |
|----------|--------|--------------|
| [README](../README.md) | ✅ Complete | Current |
| [Quick Reference](quick-reference.md) | ✅ Complete | Current |
| [Safety Design](safety-design.md) | ✅ Complete | Current |
| [Troubleshooting Guide](troubleshooting.md) | ✅ Complete | Current |
| [Error Reference](error-reference.md) | ✅ Complete | Current |
| [Configuration Guide](configuration-guide.md) | ✅ Complete | Current |

## External References

- **[ODrive Documentation](https://docs.odriverobotics.com/)** - Hardware and firmware reference
- **[ros2_control Documentation](https://control.ros.org/)** - Framework documentation
- **[Factor Robotics Wiki](https://github.com/Factor-Robotics/odrive_ros2_control/wiki/Documentation)** - Upstream project docs

## Contributing to Documentation

Found an error or have a suggestion?

1. **File an issue** describing the documentation problem
2. **Submit a PR** with your improvements
3. Follow the existing style:
   - Use Markdown
   - Include code examples
   - Link between related docs
   - Keep examples practical and tested

---

**Quick Links:**
[README](../README.md) |
[Quick Reference](quick-reference.md) |
[Safety](safety-design.md) |
[Troubleshooting](troubleshooting.md) |
[Errors](error-reference.md) |
[Configuration](configuration-guide.md)
