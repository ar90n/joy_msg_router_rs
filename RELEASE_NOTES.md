# Release Notes for v0.1.0

## 🎉 Joy Message Router v0.1.0 - Initial Release

We're excited to announce the first release of `joy_msg_router_rs`, a high-performance ROS2 node written in Rust for routing joystick inputs to various ROS messages and services.

## 🚀 Features

### Core Functionality
- **🎮 Joystick Input Processing**: Subscribes to `sensor_msgs/Joy` messages with real-time button and axis state tracking
- **📤 Universal Message Publishing**: Dynamically publishes any supported ROS message type based on configuration
- **🔧 Service Calling**: Map joystick buttons to trigger ROS service calls
- **🎯 Dynamic Modifiers**: Revolutionary modifier system allows on-the-fly control behavior changes (turbo mode, precision mode, etc.)

### Configuration System
- **⚙️ Flexible Configuration**: Support for both hierarchical YAML configuration files and flat ROS2 parameters
- **📝 Profile System**: Define multiple control profiles and switch between them
- **🔄 Unified Input Handling**: Both axes and buttons can trigger any action type

### Safety Features
- **🛡️ Enable Button**: Optional safety feature requiring a button to be held for operation
- **📊 Deadzone Filtering**: Configurable deadzones for analog inputs to prevent drift
- **🔒 Input Validation**: Comprehensive validation of configurations at startup

### Supported Message Types
- Standard messages: `Bool`, `Int16`, `Int32`, `Int64`, `Float32`, `Float64`, `String`
- Geometry messages: `Twist`, `Vector3`
- Service types: `std_srvs/srv/Trigger`, `std_srvs/srv/Empty`

### Advanced Features
- **🔥 Fire-and-forget Service Calls**: Non-blocking service calls maintain real-time responsiveness
- **📦 Runtime Type Selection**: Message types are specified in configuration, not hard-coded
- **🔄 Twist Message Accumulation**: Multiple inputs can contribute to a single Twist message
- **⏱️ Timer-based Architecture**: Efficient command queue system with configurable update rates

### Developer Experience
- **🦀 Pure Rust Implementation**: Built with `safe_drive` for memory safety and performance
- **🧪 Comprehensive Testing**: 51 unit tests ensuring reliability
- **📚 Extensive Documentation**: Complete configuration guides and examples
- **🚀 Launch Files**: Ready-to-use launch files for quick deployment
- **🔧 CI/CD Pipeline**: Automated testing, linting, and code coverage

## 📋 Example Configurations Included

- **Basic Teleoperation**: Simple joystick to cmd_vel mapping
- **Safe Teleoperation**: With deadman switch for safety
- **Advanced Control**: With modifiers for turbo mode, precision mode, and emergency stop
- **Multi-Output**: Examples of publishing to different message types
- **Service Integration**: Button mappings to service calls

## 🛠️ Technical Highlights

- Written in Rust for maximum performance and safety
- Utilizes ROS2's modern architecture
- Modular design allows easy extension
- Efficient two-pass modifier processing system
- Comprehensive error handling and logging

## 📦 Installation

```bash
# Clone into your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/ar90n/joy_msg_router_rs.git

# Build the package
cd ~/ros2_ws
colcon build --packages-select joy_msg_router_rs
source install/setup.bash
```

## 🚀 Quick Start

```bash
# Launch complete teleoperation setup
ros2 launch joy_msg_router_rs joy_teleop.launch.py

# Or with advanced features
ros2 launch joy_msg_router_rs joy_teleop.launch.py profile_name:=teleop_with_modifiers
```

## 📚 Documentation

- [Configuration Guide](docs/CONFIG.md)
- [Parameter Usage](docs/PARAMETER_USAGE.md)
- [Modifier System](docs/MODIFIER_USAGE.md)
- [Launch Files](docs/LAUNCH.md)

## 🤝 Contributing

We welcome contributions! Please feel free to submit issues and pull requests.

## 📄 License

Apache License 2.0

## 🙏 Acknowledgments

- Built with [safe_drive](https://github.com/tier4/safe_drive) - Rust bindings for ROS2
- Inspired by the need for a flexible, high-performance joystick routing solution

---

This is our first release, and we're excited to see how the community uses and extends this tool. Happy robot controlling! 🤖🎮