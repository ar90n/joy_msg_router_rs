# joy_msg_router_rs

[![CI](https://github.com/your-username/joy_msg_router_rs/workflows/CI/badge.svg)](https://github.com/your-username/joy_msg_router_rs/actions/workflows/ci.yml)
[![ROS2 Tests](https://github.com/your-username/joy_msg_router_rs/workflows/ROS2%20Integration%20Tests/badge.svg)](https://github.com/your-username/joy_msg_router_rs/actions/workflows/ros2-test.yml)
[![codecov](https://codecov.io/gh/your-username/joy_msg_router_rs/branch/main/graph/badge.svg)](https://codecov.io/gh/your-username/joy_msg_router_rs)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS](https://img.shields.io/badge/ROS-Humble-blue)](https://docs.ros.org/en/humble/)
[![Rust](https://img.shields.io/badge/Rust-1.70%2B-orange)](https://www.rust-lang.org/)

A ROS2 node written in Rust that routes joystick messages to robot control commands. This package provides flexible, configurable mapping from joystick inputs to robot movement commands with safety features like enable buttons and deadzone filtering.

## Features

- **🎮 Joystick Input Processing**: Subscribes to `sensor_msgs/Joy` messages
- **🚀 Twist Output**: Publishes `geometry_msgs/Twist` commands
- **⚙️ ROS Parameter Configuration**: Configure via ROS2 parameter system
- **🛡️ Safety Features**: Enable button support and deadzone filtering
- **🔄 Unified Input Handling**: Both axes and buttons can trigger any action
- **📦 Multiple Message Types**: Twist fields, boolean messages, and service calls
- **🧪 Comprehensive Testing**: Unit tests included
- **🚀 Launch Files**: Easy deployment with ROS2 launch system

## Quick Start

### Installation

1. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository-url> joy_msg_router_rs
```

2. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select joy_msg_router_rs
source install/setup.bash
```

### Basic Usage

#### Option 1: Complete Teleoperation Setup
```bash
# Launches both joy node and joy_msg_router with parameter file
ros2 launch joy_msg_router_rs joy_teleop.launch.py param_file:=/path/to/params.yaml
```

#### Option 2: Joy Router Only
```bash
# If you already have a joy node running
ros2 launch joy_msg_router_rs joy_router.launch.py param_file:=/path/to/params.yaml
```

#### Option 3: Direct Node Execution
```bash
# Run with parameter file
ros2 run joy_msg_router_rs joy_msg_router --ros-args --params-file /path/to/params.yaml

# Or set parameters individually
ros2 run joy_msg_router_rs joy_msg_router --ros-args \
  -p profile_name:=teleop_safe \
  -p enable_button:=4 \
  -p input_mappings.0.source_type:=axis \
  -p input_mappings.0.source_index:=1 \
  -p input_mappings.0.action_type:=publish_twist_field \
  -p input_mappings.0.field:=linear_x \
  -p input_mappings.0.scale:=0.5
```

## Configuration

The node uses ROS2 parameters for configuration. See `config/params_example.yaml` for a complete example.

### Parameter Structure

```yaml
joy_msg_router:
  ros__parameters:
    # Profile name (for identification)
    profile_name: "teleop_safe"
    
    # Optional: Enable button that must be held
    enable_button: 4  # L1/LB button
    
    # Input mappings (unified for axes and buttons)
    # Format: input_mappings.N.parameter_name
    
    # Example axis mapping
    input_mappings.0.source_type: "axis"
    input_mappings.0.source_index: 1
    input_mappings.0.action_type: "publish_twist_field"
    input_mappings.0.field: "linear_x"
    input_mappings.0.scale: 0.5
    input_mappings.0.offset: 0.0
    input_mappings.0.deadzone: 0.1
    
    # Example button mapping
    input_mappings.1.source_type: "button"
    input_mappings.1.source_index: 0
    input_mappings.1.action_type: "publish_bool"
    input_mappings.1.topic: "/lights/enable"
    input_mappings.1.value: true
    input_mappings.1.once: true
```

### Supported Action Types

1. **publish_twist_field**: Publish to a specific field of the Twist message
   - `field`: One of "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"
   - Works with both axes (scaled values) and buttons (fixed values)

2. **publish_bool**: Publish boolean messages
   - `topic`: Target topic name
   - `value`: Boolean value to publish
   - `once`: If true, publish only on activation; if false, publish continuously while active

3. **call_service**: Call a ROS service
   - `service_name`: Name of the service to call
   - `service_type`: Service type (e.g., "std_srvs/srv/Trigger")

### Input Processing

- **Axes**: Analog inputs with deadzone, scaling, and offset
  - `scale`: Multiplier for the raw value
  - `offset`: Added after scaling
  - `deadzone`: Values below this threshold are treated as zero

- **Buttons**: Digital inputs that can trigger any action
  - For twist fields: Uses scale/offset to set fixed values
  - For bool/service: Triggers on press or continuously

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joy` | `sensor_msgs/Joy` | Input joystick messages |
| `/cmd_vel` | `geometry_msgs/Twist` | Output velocity commands |
| Custom topics | `std_msgs/Bool` | As configured in publish_bool actions |

## Example Configurations

### PlayStation 4 Controller
See `config/examples/ps4_controller.yaml`

### Xbox Controller  
See `config/examples/xbox_controller.yaml`

### Custom Parameter File
See `config/params_example.yaml`

## Development

### Running Tests
```bash
# Rust unit tests
cd ~/ros2_ws/src/joy_msg_router_rs
cargo test

# ROS2 integration tests
cd ~/ros2_ws
colcon test --packages-select joy_msg_router_rs
```

### Code Quality
```bash
# Format code
cargo fmt

# Run linter
cargo clippy -- -D warnings
```

## License

Apache License 2.0

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.