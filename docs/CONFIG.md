# Joy Message Router Configuration

This document describes the hierarchical YAML configuration format for the joy_msg_router node.

## Configuration File Format

Configuration files use hierarchical YAML format to define profiles for routing joystick inputs to ROS messages, services, and modifiers.

### Basic Structure

```yaml
# Default profile to use when none is specified
default_profile: "profile_name"

profiles:
  profile_name:
    # Optional: button that must be held to enable movement
    enable_button: 4
    
    # Unified input mappings for all sources and actions
    input_mappings:
      - id: "optional_id"        # Optional: unique ID for modifier targeting
        source_type: axis        # axis or button
        source_index: 1          # 0-based index
        scale: 0.5               # Value multiplier
        offset: 0.0              # Value offset
        deadzone: 0.1            # Threshold for axes
        action:
          type: publish          # publish, call_service, or modifier
          # Action-specific fields...
```

### Action Types

#### 1. Publish Action
Publishes values to ROS topics with any message type:

```yaml
action:
  type: publish
  topic: "/cmd_vel"
  message_type: "geometry_msgs/msg/Twist"
  field: "linear.x"    # Optional: specific field to set
  once: false          # Optional: publish once or continuously
```

Supported message types include:
- `geometry_msgs/msg/Twist` - with fields like `linear.x`, `angular.z`
- `std_msgs/msg/Bool`, `std_msgs/msg/Int32`, `std_msgs/msg/Float64`, `std_msgs/msg/String`
- Any other message type supported by the ROS2 system

#### 2. Service Call Action
Calls ROS services when triggered:

```yaml
action:
  type: call_service
  service_name: "/reset_system"
  service_type: "std_srvs/srv/Trigger"
```

#### 3. Modifier Action
Dynamically modifies other mappings' parameters:

```yaml
action:
  type: modifier
  targets:
    # Target by mapping ID
    - mapping_id: "forward_movement"
    # Or target by source
    - source:
        type: axis
        index: 1
  scale_multiplier: 2.0      # Multiply target's scale
  offset_delta: 0.1          # Add to target's offset
  deadzone_override: 0.05    # Override target's deadzone
  apply_gradually: true      # For axes: interpolate by value
```

### Field Descriptions

#### Profile Fields
- `enable_button`: Button index that must be held to enable the profile (optional)
- `input_mappings`: List of input-to-action mappings

#### Input Mapping Fields
- `id`: Optional unique identifier for modifier targeting
- `source_type`: Either "axis" or "button"
- `source_index`: 0-based index of the input
- `scale`: Multiplier applied to the input value (default: 1.0)
- `offset`: Value added after scaling (default: 0.0)
- `deadzone`: For axes, values below this are treated as zero (default: 0.1)
- `action`: The action to perform

### Multiple Profiles

Define multiple profiles in a single file:

```yaml
profiles:
  teleop:
    # Basic teleoperation setup
    
  teleop_safe:
    enable_button: 4
    # Safe mode with deadman switch
    
  teleop_with_modifiers:
    # Advanced setup with turbo/precision modes
```

## Usage Examples

### Loading a Configuration File

```bash
# Using ros2 run
ros2 run joy_msg_router_rs joy_msg_router --ros-args -p config_file:=/path/to/config.yaml

# With specific profile
ros2 run joy_msg_router_rs joy_msg_router --ros-args \
  -p config_file:=config.yaml -p profile_name:=teleop_with_modifiers

# Using launch file
ros2 launch joy_msg_router_rs joy_router.launch.py \
  config_file:=./config/default.yaml profile_name:=teleop_safe
```

### Example Configuration

See `/config/default.yaml` for a complete example including:
- Basic teleoperation
- Safe mode with deadman switch
- Modifier support (turbo, precision, emergency stop)
- Multiple output message types

## Tips

1. **Modifier Stacking**: Multiple modifiers can affect the same mapping
   - Scale multipliers are multiplicative
   - Offset deltas are additive
   - Deadzone overrides use the minimum value

2. **Gradual Modifiers**: Set `apply_gradually: true` for axis-based modifiers to interpolate based on axis position

3. **Safety**: Use `enable_button` for profiles that control movement

4. **Organization**: Use meaningful IDs for mappings that will be targeted by modifiers