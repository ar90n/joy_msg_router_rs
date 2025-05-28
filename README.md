# joy_msg_router_rs

A ROS2 node written in Rust that routes joystick messages to robot control commands. This package provides flexible, configurable mapping from joystick inputs to robot movement commands with safety features like enable buttons and deadzone filtering.

## Features

- **🎮 Joystick Input Processing**: Subscribes to `sensor_msgs/Joy` messages
- **🚀 Twist Output**: Publishes `geometry_msgs/Twist` commands
- **⚙️ Configurable Profiles**: YAML-based configuration for different robot types
- **🛡️ Safety Features**: Enable button support and deadzone filtering
- **🔲 Button Actions**: Trigger fixed movements or service calls with button presses
- **📦 Multiple Profiles**: Pre-configured profiles for teleop, holonomic, and drone control
- **🧪 Comprehensive Testing**: Unit tests and integration tests included
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
# Launches both joy node and joy_msg_router
ros2 launch joy_msg_router_rs joy_teleop.launch.py
```

#### Option 2: Joy Router Only
```bash
# If you already have a joy node running
ros2 launch joy_msg_router_rs joy_router.launch.py
```

#### Option 3: Direct Node Execution
```bash
# Run with default hardcoded configuration
ros2 run joy_msg_router_rs joy_msg_router
```

## Configuration

### Available Profiles

The package includes several pre-configured profiles:

- **`teleop`**: Basic differential drive robot (default)
- **`teleop_safe`**: Same as teleop but requires enable button (L1)
- **`holonomic`**: For omnidirectional robots with strafe capability
- **`drone`**: For UAV/drone control with 3D movement

### Using Different Profiles

```bash
# Safe teleoperation with deadman switch
ros2 launch joy_msg_router_rs joy_teleop.launch.py profile:=teleop_safe

# Holonomic robot control
ros2 launch joy_msg_router_rs joy_teleop.launch.py profile:=holonomic

# Drone/UAV control
ros2 launch joy_msg_router_rs joy_teleop.launch.py profile:=drone
```

### Custom Configuration

Create your own configuration file based on `config/default.yaml`:

```bash
# Use custom config file
ros2 launch joy_msg_router_rs joy_router.launch.py \
  config_file:=/path/to/my_config.yaml \
  profile:=my_custom_profile
```

## Configuration Format

The configuration uses YAML format with the following structure:

```yaml
default_profile: "teleop"
profiles:
  my_profile:
    # Optional: Enable button (joystick button that must be held)
    enable_button: 4  # L1 button
    
    # Axis mappings: joystick axes to robot movement
    axis_mappings:
      - joy_axis: 1              # Left stick Y-axis
        output_field: linear_x   # Forward/backward movement
        scale: 0.5               # Maximum speed (m/s)
        offset: 0.0              # Offset after scaling
        deadzone: 0.1            # Values below this are ignored
    
    # Button mappings: buttons to discrete actions
    button_mappings:
      - button: 0
        action:
          type: publish_twist    # Publish fixed Twist message
          linear_x: 0.0
          angular_z: 0.0
      
      - button: 1
        action:
          type: call_service     # Call ROS service (planned feature)
          service_name: "/emergency_stop"
          service_type: "std_srvs/srv/Trigger"
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joy` | `sensor_msgs/Joy` | Input joystick messages |
| `/cmd_vel` | `geometry_msgs/Twist` | Output velocity commands |

## Parameters

The node supports configuration through multiple methods with the following priority:

1. **Command line arguments** (highest priority)
2. **Environment variables**  
3. **Default values** (lowest priority)

### Command Line Arguments
```bash
ros2 run joy_msg_router_rs joy_msg_router --config /path/to/config.yaml --profile drone
```

### Environment Variables
```bash
export JOY_ROUTER_CONFIG_FILE=/path/to/config.yaml
export JOY_ROUTER_PROFILE=holonomic
ros2 run joy_msg_router_rs joy_msg_router
```

### Available Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `--config` | string | "config/default.yaml" | Path to configuration file |
| `--profile` | string | "teleop" | Profile to use from config |
| `JOY_ROUTER_CONFIG_FILE` | env var | - | Environment variable for config file |
| `JOY_ROUTER_PROFILE` | env var | - | Environment variable for profile |

## Architecture

The joy_msg_router node processes joystick input through several stages:

1. **Joy Message Reception**: Subscribes to `/joy` topic
2. **Button State Tracking**: Tracks button press/release events
3. **Enable Button Check**: Verifies safety conditions
4. **Button Action Processing**: Handles discrete button actions
5. **Axis-to-Twist Conversion**: Maps joystick axes to robot movement
6. **Twist Publishing**: Outputs velocity commands

## Development

### Building from Source

```bash
# Install dependencies
sudo apt install ros-${ROS_DISTRO}-joy

# Clone and build
cd ~/ros2_ws/src
git clone <repository-url> joy_msg_router_rs
cd ~/ros2_ws
colcon build --packages-select joy_msg_router_rs
```

### Running Tests

```bash
# Unit tests (Rust)
cd src/joy_msg_router_rs
cargo test

# Integration tests (Python)
source install/setup.bash
python3 src/joy_msg_router_rs/tests/test_simple.py

# All tests via colcon
colcon test --packages-select joy_msg_router_rs
```

### Code Structure

```
src/joy_msg_router_rs/
├── src/
│   ├── main.rs              # Main node implementation
│   ├── config.rs            # Configuration data structures  
│   ├── config_loader.rs     # YAML configuration loading
│   └── button_tracker.rs    # Button state management
├── config/
│   └── default.yaml         # Default configuration profiles
├── launch/
│   ├── joy_router.launch.py # Joy router only
│   └── joy_teleop.launch.py # Complete teleoperation setup
└── tests/
    ├── test_simple.py       # Basic functionality tests
    ├── test_integration.py  # Launch-based integration tests
    └── test_joy_router.py   # Comprehensive node tests
```

## Troubleshooting

### Joystick Not Working
```bash
# Check joystick device
ls /dev/input/js*
jstest /dev/input/js0

# Set permissions
sudo chmod 666 /dev/input/js0
```

### No cmd_vel Output
```bash
# Check if enable button is required and pressed
ros2 topic echo /joy

# Verify deadzone settings
ros2 param get /joy_msg_router profile

# Monitor output
ros2 topic echo /cmd_vel
```

### Custom Configuration Issues
```bash
# Validate YAML syntax
python3 -c "import yaml; yaml.safe_load(open('config.yaml'))"

# Check node logs
ros2 run joy_msg_router_rs joy_msg_router --ros-args --log-level debug
```

## Examples

### Multi-Robot Setup
```bash
# Robot 1
ros2 launch joy_msg_router_rs joy_teleop.launch.py \
  namespace:=robot1 \
  cmd_vel_topic:=/robot1/cmd_vel \
  device:=/dev/input/js0

# Robot 2
ros2 launch joy_msg_router_rs joy_teleop.launch.py \
  namespace:=robot2 \
  cmd_vel_topic:=/robot2/cmd_vel \
  device:=/dev/input/js1
```

### Custom Topic Names
```bash
ros2 launch joy_msg_router_rs joy_router.launch.py \
  joy_topic:=/custom_joy \
  cmd_vel_topic:=/robot/base/cmd_vel
```

### Parameter Configuration Examples
```bash
# Use command line arguments
ros2 run joy_msg_router_rs joy_msg_router --profile drone --config /custom/config.yaml

# Use environment variables
export JOY_ROUTER_PROFILE=holonomic
ros2 run joy_msg_router_rs joy_msg_router

# Combine launch files with parameters
ros2 launch joy_msg_router_rs joy_teleop.launch.py profile:=teleop_safe

# Runtime profile switching (restart required)
ros2 run joy_msg_router_rs joy_msg_router --profile teleop_safe
# Kill and restart with different profile
ros2 run joy_msg_router_rs joy_msg_router --profile drone
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass: `cargo test && python3 tests/test_simple.py`
5. Submit a pull request

## License

This project is licensed under the Apache License 2.0.

## Acknowledgments

- Built with [safe_drive](https://github.com/tier4/safe_drive) - A safe Rust wrapper for ROS2
- Inspired by the standard `joy_teleop` package
- Part of the ROS2 ecosystem