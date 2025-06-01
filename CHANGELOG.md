# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2025-01-06

### Added
- Initial release of joy_msg_router_rs
- Core joystick input processing from `sensor_msgs/Joy` messages
- Universal message publishing system supporting multiple ROS2 message types
  - Standard messages: Bool, Int16, Int32, Int64, Float32, Float64, String
  - Geometry messages: Twist, Vector3
- Service calling functionality for buttons
  - Support for std_srvs/srv/Trigger and std_srvs/srv/Empty
- Dynamic modifier system for runtime control behavior changes
  - Scale multipliers, offset deltas, and deadzone overrides
  - Gradual modifiers based on axis values
  - Target mappings by ID or source characteristics
- Hierarchical YAML configuration file support
- ROS2 parameter system integration
- Profile-based configuration system
- Enable button safety feature
- Deadzone filtering for analog inputs
- Unified input handling (axes and buttons can trigger any action)
- Twist message accumulation from multiple inputs
- Timer-based command queue architecture
- Comprehensive error handling and logging system
- Launch files for easy deployment
  - joy_router.launch.py for standalone router
  - joy_teleop.launch.py for complete teleoperation setup
- Example configurations for various use cases
- Full documentation suite
- Comprehensive test coverage (51 unit tests)
- CI/CD pipeline with GitHub Actions

### Technical Details
- Built with Rust and safe_drive library
- Non-blocking service calls for real-time performance
- Runtime message type selection
- Two-pass modifier processing system
- Efficient command queue with configurable update rates

[0.1.0]: https://github.com/ar90n/joy_msg_router_rs/releases/tag/v0.1.0