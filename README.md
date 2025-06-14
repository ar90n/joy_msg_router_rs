# joy_msg_router_rs

[![CI](https://github.com/ar90n/joy_msg_router_rs/workflows/CI/badge.svg)](https://github.com/ar90n/joy_msg_router_rs/actions/workflows/ci.yml)
[![Code Coverage](https://github.com/ar90n/joy_msg_router_rs/workflows/Code%20Coverage/badge.svg)](https://github.com/ar90n/joy_msg_router_rs/actions/workflows/coverage.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS](https://img.shields.io/badge/ROS-Humble-blue)](https://docs.ros.org/en/humble/)
[![Rust](https://img.shields.io/badge/Rust-1.70%2B-orange)](https://www.rust-lang.org/)
![Built with vibe coding](https://img.shields.io/badge/built%20with-vibe%20coding-ff69b4)

A ROS2 node written in Rust that routes joystick (Joy) messages to various ROS messages and services. This package provides flexible, configurable mapping from joystick inputs to any supported ROS message type with safety features like enable buttons and deadzone filtering.

## Features

- **🎮 Joystick Input Processing**: Subscribes to `sensor_msgs/Joy` messages
- **📤 Universal Message Publishing**: Publishes any supported ROS message type dynamically
- **🔧 Service Calling**: Map buttons to ROS service calls
- **🎯 Dynamic Modifiers**: Change control behavior on-the-fly (turbo, precision modes)
- **⚙️ Flexible Configuration**: Hierarchical YAML profiles or flat ROS2 parameters
- **🛡️ Safety Features**: Enable button support and deadzone filtering
- **🔄 Unified Input Handling**: Both axes and buttons can trigger any action
- **📦 Runtime Type Selection**: Message types specified in configuration
- **🧪 Comprehensive Testing**: Unit and integration tests included
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
# Launches both joy node and joy_msg_router with default configuration
ros2 launch joy_msg_router_rs joy_teleop.launch.py

# Use a specific profile from the default config
ros2 launch joy_msg_router_rs joy_teleop.launch.py profile_name:=teleop_with_modifiers

# Use a custom config file
ros2 launch joy_msg_router_rs joy_teleop.launch.py config_file:=/path/to/config.yaml
```

#### Option 2: Joy Router Only
```bash
# If you already have a joy node running
ros2 launch joy_msg_router_rs joy_router.launch.py

# With custom configuration
ros2 launch joy_msg_router_rs joy_router.launch.py config_file:=/path/to/config.yaml profile_name:=my_profile
```

#### Option 3: Direct Node Execution
```bash
# Run with hierarchical config file (recommended)
ros2 run joy_msg_router_rs joy_msg_router --ros-args -p config_file:=/path/to/config.yaml

# Specify a profile from the config file
ros2 run joy_msg_router_rs joy_msg_router --ros-args \
  -p config_file:=config.yaml -p profile_name:=teleop_with_modifiers

# Or use flat ROS2 parameters directly
ros2 run joy_msg_router_rs joy_msg_router --ros-args \
  -p profile_name:=custom \
  -p enable_button:=4 \
  -p input_mappings.0.source_type:=axis \
  -p input_mappings.0.source_index:=1 \
  -p input_mappings.0.action_type:=publish \
  -p input_mappings.0.topic:=/cmd_vel \
  -p input_mappings.0.message_type:="geometry_msgs/msg/Twist" \
  -p input_mappings.0.field:="linear.x" \
  -p input_mappings.0.scale:=0.5
```

## Configuration

The node can be configured using either:
1. **Hierarchical YAML files** (recommended): See `config/default.yaml` for examples
2. **ROS2 parameters**: Use flat parameter format for runtime configuration

For detailed documentation, see the [docs](docs/) directory.

### YAML Profile Format (Recommended)

```yaml
# Profile-based configuration
default_profile: "teleop"

profiles:
  teleop:
    enable_button: 4  # Optional: button that must be held
    
    input_mappings:
      # Axis example
      - source_type: axis
        source_index: 1
        scale: 0.5
        offset: 0.0
        deadzone: 0.1
        action:
          type: publish
          topic: "/cmd_vel"
          message_type: "geometry_msgs/msg/Twist"
          field: "linear.x"
      
      # Button example
      - source_type: button
        source_index: 0
        scale: 1.0
        action:
          type: publish
          topic: "/lights/enable"
          message_type: "std_msgs/msg/Bool"
          once: true
```

### ROS2 Parameter Structure

```yaml
joy_msg_router:
  ros__parameters:
    # Profile name (for identification)
    profile_name: "teleop_safe"
    
    # Optional: Enable button that must be held
    enable_button: 4  # L1/LB button
    
    # Input mappings (unified for axes and buttons)
    # Format: input_mappings.N.parameter_name
    
    # Example axis mapping (Twist message)
    input_mappings.0.source_type: "axis"
    input_mappings.0.source_index: 1
    input_mappings.0.action_type: "publish"
    input_mappings.0.topic: "/cmd_vel"
    input_mappings.0.message_type: "geometry_msgs/msg/Twist"
    input_mappings.0.field: "linear.x"  # Nested field support
    input_mappings.0.scale: 0.5
    input_mappings.0.offset: 0.0
    input_mappings.0.deadzone: 0.1
    
    # Example button mapping (Bool message)
    input_mappings.1.source_type: "button"
    input_mappings.1.source_index: 0
    input_mappings.1.action_type: "publish"
    input_mappings.1.topic: "/lights/enable"
    input_mappings.1.message_type: "std_msgs/msg/Bool"
    input_mappings.1.scale: 1.0  # Non-zero = true
    input_mappings.1.once: true
    
    # Example service call
    input_mappings.2.source_type: "button"
    input_mappings.2.source_index: 2
    input_mappings.2.action_type: "call_service"
    input_mappings.2.service_name: "/reset_odometry"
    input_mappings.2.service_type: "std_srvs/srv/Trigger"
```

### Supported Action Types

1. **publish**: Publish any supported ROS message type
   - `topic`: Target topic name
   - `message_type`: Full ROS message type (e.g., "std_msgs/msg/Float64", "geometry_msgs/msg/Twist")
   - `field`: Optional field name for multi-field messages (e.g., "x" for Vector3, "linear.x" for Twist)
   - `once`: If true, publish only on activation; if false, publish continuously while active
   - Supported types:
     - `std_msgs/msg/Bool`, `Int16`, `Int32`, `Int64`, `Float32`, `Float64`, `String`
     - `geometry_msgs/msg/Twist`, `Vector3`
   - Note: For Twist messages, values are accumulated from all active mappings before publishing

2. **call_service**: Call a ROS service (button only)
   - `service_name`: Name of the service to call (required)
   - `service_type`: Service type (required)
   - Supported service types:
     - `std_srvs/srv/Trigger`: Service with boolean success and message response
     - `std_srvs/srv/Empty`: Service with no request/response data
   - Services are called when the button is pressed (on rising edge)

3. **modifier**: Dynamically modify other mappings' parameters
   - `targets`: List of target specifications (mapping ID or source characteristics)
   - `scale_multiplier`: Multiply target's scale value
   - `offset_delta`: Add to target's offset value
   - `deadzone_override`: Override target's deadzone
   - `apply_gradually`: For axis modifiers, interpolate based on axis position
   - See [MODIFIER_USAGE.md](docs/MODIFIER_USAGE.md) for detailed examples

### Input Processing

- **Axes**: Analog inputs with deadzone, scaling, and offset
  - `scale`: Multiplier for the raw value
  - `offset`: Added after scaling
  - `deadzone`: Values below this threshold are treated as zero

- **Buttons**: Digital inputs that can trigger any action
  - For numeric types: Uses scale/offset to set fixed values when pressed
  - For Bool messages: Non-zero scale value = true, zero = false
  - For services: Triggers on button press
  - `once` parameter controls single vs continuous publishing

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joy` | `sensor_msgs/Joy` | Input joystick messages |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| Configured topics | Any supported type | Topics are configured per mapping |
| Common: `/cmd_vel` | `geometry_msgs/Twist` | Robot velocity commands |

### Services

| Service | Type | Description |
|---------|------|-------------|
| Configured services | `std_srvs/srv/Trigger` | Services called as configured |

## Example Configurations

The default configuration file `config/default.yaml` includes several example profiles:
- **teleop**: Basic teleoperation without safety features
- **teleop_safe**: Teleoperation with deadman switch (L1 button must be held)
- **teleop_with_modifiers**: Advanced setup with turbo mode (R1), precision mode (L1), and emergency stop (X)
- **multi_output**: Examples of publishing to different message types

For flat ROS2 parameter format examples, see:
- `config/default_params.yaml` - Basic flat parameter format
- `config/teleop_safe_params.yaml` - Safe teleoperation parameters
- `config/teleop_with_modifiers_params.yaml` - Parameters with modifier support

## Architecture

This node is built using:
- **safe_drive**: Rust bindings for ROS2
- **Runtime message type selection**: Publishers are created dynamically based on configuration
- **Unified input processing**: Single processing loop handles all input types
- **Two-pass modifier processing**: Collect active modifiers, then apply to mappings
- **Accumulation for Twist messages**: Multiple inputs can contribute to a single Twist message
- **Fire-and-forget service calling**: Service calls don't block the main loop, ensuring real-time joystick responsiveness

### Key Components
1. **JoyMsgTracker**: Tracks joystick state and detects changes
2. **Publishers**: Manages all configured publishers with runtime type selection
3. **ServiceClients**: Handles service client creation and calls
4. **Profile**: Configuration structure supporting both hierarchical YAML and flat ROS parameters
5. **Modifier System**: Dynamic parameter modification based on joystick input

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

## Troubleshooting

### Common Issues

1. **No output on /cmd_vel**
   - Check if enable_button is configured and being held
   - Verify joystick is publishing to /joy: `ros2 topic echo /joy`
   - Check deadzone settings - they might be filtering out input

2. **Service calls not working**
   - Currently only `std_srvs/srv/Trigger` services are supported
   - Ensure both service_name and service_type are specified

3. **Message type not supported**
   - Check the supported types list in the action type documentation
   - Ensure message_type uses full format: `package/msg/Type`

4. **Parameter loading issues**
   - For YAML profiles: Place file in config directory and use correct format
   - For ROS parameters: Ensure proper namespacing under `joy_msg_router.ros__parameters`

## License

Apache License 2.0

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.