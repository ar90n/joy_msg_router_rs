# Joy Message Router Documentation

This directory contains documentation for the joy_msg_router_rs ROS2 node.

## Documentation Index

### Configuration
- [CONFIG.md](CONFIG.md) - Configuration file format and structure
- [PARAMETER_USAGE.md](PARAMETER_USAGE.md) - ROS2 parameter system usage
- [MODIFIER_USAGE.md](MODIFIER_USAGE.md) - Modifier action type guide

### Usage
- [LAUNCH.md](LAUNCH.md) - Launch file usage and examples

### Development
- [TESTING.md](TESTING.md) - Testing guide and test descriptions

## Quick Start

1. **Basic Usage**
   ```bash
   ros2 run joy_msg_router_rs joy_msg_router --ros-args -p config_file:=/path/to/config.yaml
   ```

2. **Using Launch File**
   ```bash
   ros2 launch joy_msg_router_rs joy_router.launch.py
   ```

3. **With Specific Profile**
   ```bash
   ros2 run joy_msg_router_rs joy_msg_router --ros-args -p config_file:=config.yaml -p profile_name:=teleop_with_modifiers
   ```

## Key Features

- **Message Routing**: Route joystick inputs to various ROS2 message types
- **Service Calls**: Trigger ROS2 services from button presses
- **Modifiers**: Dynamically modify control behavior (scale, offset, deadzone)
- **Profile System**: Switch between different control configurations
- **Hierarchical Config**: Use readable YAML configuration files
- **Enable Button**: Optional safety feature requiring button hold for operation

## Configuration Overview

The node supports both hierarchical YAML configuration files and flat ROS2 parameters:

- **Hierarchical YAML** (preferred): See [CONFIG.md](CONFIG.md)
- **ROS2 Parameters**: See [PARAMETER_USAGE.md](PARAMETER_USAGE.md)

## Action Types

1. **publish**: Send values to ROS2 topics
2. **call_service**: Call ROS2 services
3. **modifier**: Dynamically modify other mappings

See specific documentation files for detailed usage instructions.