# Launch Files

This directory contains launch files for the joy_msg_router package.

## Available Launch Files

### joy_router.launch.py
Launches only the joy_msg_router node. Useful when you already have a joy node running or want to use a different joystick driver.

**Usage:**
```bash
# Basic launch with default configuration
ros2 launch joy_msg_router_rs joy_router.launch.py

# Launch with specific profile
ros2 launch joy_msg_router_rs joy_router.launch.py profile:=teleop_safe

# Launch with custom configuration file
ros2 launch joy_msg_router_rs joy_router.launch.py config_file:=/path/to/config.yaml

# Launch with custom topic names
ros2 launch joy_msg_router_rs joy_router.launch.py joy_topic:=/custom_joy cmd_vel_topic:=/robot/cmd_vel

# Launch in a namespace
ros2 launch joy_msg_router_rs joy_router.launch.py namespace:=robot1
```

**Parameters:**
- `config_file`: Path to configuration file (default: uses package default)
- `profile`: Profile to use from config file (default: "teleop")
- `namespace`: Namespace for the node (default: "")
- `joy_topic`: Input joy topic (default: "/joy")
- `cmd_vel_topic`: Output twist topic (default: "/cmd_vel")
- `use_default_config`: Whether to use package default config (default: "true")

### joy_teleop.launch.py
Complete teleoperation setup that launches both the joy node and joy_msg_router node.

**Usage:**
```bash
# Basic teleoperation setup
ros2 launch joy_msg_router_rs joy_teleop.launch.py

# Use different joystick device
ros2 launch joy_msg_router_rs joy_teleop.launch.py device:=/dev/input/js1

# Use safe teleoperation profile with deadman switch
ros2 launch joy_msg_router_rs joy_teleop.launch.py profile:=teleop_safe

# Holonomic robot control
ros2 launch joy_msg_router_rs joy_teleop.launch.py profile:=holonomic

# Drone/UAV control
ros2 launch joy_msg_router_rs joy_teleop.launch.py profile:=drone

# Multi-robot setup
ros2 launch joy_msg_router_rs joy_teleop.launch.py namespace:=robot1 cmd_vel_topic:=/robot1/cmd_vel
```

**Parameters:**
- `device`: Joystick device file (default: "/dev/input/js0")
- `profile`: Joy router profile (default: "teleop")
- `namespace`: Namespace for all nodes (default: "")
- `cmd_vel_topic`: Output velocity topic (default: "/cmd_vel")
- `deadzone`: Joystick deadzone (default: "0.1")
- `autorepeat_rate`: Joy message repeat rate in Hz (default: "20.0")

## Examples

### Basic Robot Teleoperation
```bash
ros2 launch joy_msg_router_rs joy_teleop.launch.py
```

### Safe Teleoperation with Deadman Switch
```bash
ros2 launch joy_msg_router_rs joy_teleop.launch.py profile:=teleop_safe
```

### Multi-Robot Control
```bash
# Terminal 1 - Robot 1
ros2 launch joy_msg_router_rs joy_teleop.launch.py \
  namespace:=robot1 \
  cmd_vel_topic:=/robot1/cmd_vel \
  device:=/dev/input/js0

# Terminal 2 - Robot 2  
ros2 launch joy_msg_router_rs joy_teleop.launch.py \
  namespace:=robot2 \
  cmd_vel_topic:=/robot2/cmd_vel \
  device:=/dev/input/js1
```

### Custom Configuration
```bash
ros2 launch joy_msg_router_rs joy_router.launch.py \
  config_file:=/path/to/my_robot_config.yaml \
  profile:=my_custom_profile
```

## Troubleshooting

### Joystick Not Detected
- Check that your joystick is connected: `ls /dev/input/js*`
- Verify permissions: `sudo chmod 666 /dev/input/js0`
- Test with `jstest /dev/input/js0`

### No cmd_vel Messages
- Check if enable button is configured and pressed (for teleop_safe profile)
- Verify joystick values are above deadzone threshold
- Use `ros2 topic echo /joy` to see raw joystick data
- Use `ros2 topic echo /cmd_vel` to see output

### Custom Profiles
Create your own configuration file based on `config/default.yaml` and specify it with the `config_file` parameter.