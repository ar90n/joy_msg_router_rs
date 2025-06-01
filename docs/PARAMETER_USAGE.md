# ROS2 Parameter Usage

The joy_msg_router_rs node supports configuration through ROS2 parameters in two ways:
1. **Hierarchical YAML configuration files** (recommended)
2. **Flat ROS2 parameters** (for runtime configuration)

## Configuration Methods

### Method 1: Using Configuration Files (Recommended)

The node can load hierarchical YAML configuration files using the `config_file` parameter:

```bash
ros2 run joy_msg_router_rs joy_msg_router --ros-args -p config_file:=/path/to/config.yaml
```

You can also specify a profile from the configuration file:

```bash
ros2 run joy_msg_router_rs joy_msg_router --ros-args \
  -p config_file:=/path/to/config.yaml \
  -p profile_name:=teleop_with_modifiers
```

### Method 2: Direct ROS2 Parameters

For runtime configuration or when not using a config file, you can specify parameters directly:

```bash
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

## Available Parameters

### Configuration Selection
- **`config_file`** (string): Path to hierarchical YAML configuration file
- **`profile_name`** (string): Name of the profile to use (required if not using config_file)

### Profile Parameters
- **`enable_button`** (integer): Button that must be held to enable the profile

### Input Mapping Parameters

For each mapping (indexed from 0):
- **`input_mappings.N.id`** (string): Optional unique identifier
- **`input_mappings.N.source_type`** (string): "axis" or "button"
- **`input_mappings.N.source_index`** (integer): 0-based index
- **`input_mappings.N.scale`** (double): Scale factor (default: 1.0)
- **`input_mappings.N.offset`** (double): Offset value (default: 0.0)
- **`input_mappings.N.deadzone`** (double): Deadzone threshold (default: 0.1)
- **`input_mappings.N.action_type`** (string): "publish", "call_service", or "modifier"

#### For Publish Actions:
- **`input_mappings.N.topic`** (string): Target topic
- **`input_mappings.N.message_type`** (string): Message type (e.g., "geometry_msgs/msg/Twist")
- **`input_mappings.N.field`** (string): Optional field name
- **`input_mappings.N.once`** (boolean): Publish once or continuously

#### For Service Actions:
- **`input_mappings.N.service_name`** (string): Service name
- **`input_mappings.N.service_type`** (string): Service type

#### For Modifier Actions:
- **`input_mappings.N.targets.M.mapping_id`** (string): Target by ID
- **`input_mappings.N.targets.M.source_type`** (string): Target by source type
- **`input_mappings.N.targets.M.source_index`** (integer): Target by source index
- **`input_mappings.N.scale_multiplier`** (double): Scale multiplier
- **`input_mappings.N.offset_delta`** (double): Offset delta
- **`input_mappings.N.deadzone_override`** (double): Deadzone override
- **`input_mappings.N.apply_gradually`** (boolean): Gradual application

## Complete Example

### Flat Parameter File Example

```yaml
joy_msg_router:
  ros__parameters:
    profile_name: "custom_teleop"
    enable_button: 4
    
    # Forward/backward movement
    input_mappings.0.id: "forward"
    input_mappings.0.source_type: "axis"
    input_mappings.0.source_index: 1
    input_mappings.0.scale: 0.5
    input_mappings.0.deadzone: 0.1
    input_mappings.0.action_type: "publish"
    input_mappings.0.topic: "/cmd_vel"
    input_mappings.0.message_type: "geometry_msgs/msg/Twist"
    input_mappings.0.field: "linear.x"
    
    # Turbo modifier
    input_mappings.1.source_type: "button"
    input_mappings.1.source_index: 5
    input_mappings.1.action_type: "modifier"
    input_mappings.1.targets.0.mapping_id: "forward"
    input_mappings.1.scale_multiplier: 2.0
```

### Launch File Integration

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy_msg_router_rs',
            executable='joy_msg_router',
            name='joy_msg_router',
            parameters=[{
                'config_file': '/path/to/config.yaml',
                'profile_name': 'teleop_with_modifiers'
            }]
        )
    ])
```

## Tips

1. **Config File vs Parameters**: Use config files for complex setups, direct parameters for simple cases
2. **Parameter Priority**: If both config_file and individual parameters are set, config_file takes precedence
3. **Validation**: The node validates all parameters at startup and reports errors clearly
4. **Modifiers**: When using modifiers via parameters, ensure target mappings have IDs or use source targeting