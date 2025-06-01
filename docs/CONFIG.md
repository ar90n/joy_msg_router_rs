# Joy Message Router Configuration

This directory contains configuration files for the joy_msg_router node.

## Configuration File Format

Configuration files use YAML format and define profiles for routing joystick inputs to ROS messages.

### Basic Structure

```yaml
default_profile: "profile_name"

profiles:
  profile_name:
    enable_button: -1  # Optional: button index that must be held (-1 for always enabled)
    
    axis_mappings:
      - joy_axis: 1
        output_field: linear_x
        scale: 0.5
        offset: 0.0
        deadzone: 0.1
    
    button_mappings:
      - button: 0
        action:
          type: publish_twist
          linear_x: 0.0
          # ... other twist fields
```

### Field Descriptions

#### Profile Fields
- `enable_button`: Button index that must be held to enable output. Use -1 or omit for always enabled.
- `axis_mappings`: List of axis-to-field mappings
- `button_mappings`: List of button-to-action mappings

#### Axis Mapping Fields
- `joy_axis`: Index of the joystick axis (0-based)
- `output_field`: Target field in the output message. Options:
  - `linear_x`, `linear_y`, `linear_z`
  - `angular_x`, `angular_y`, `angular_z`
- `scale`: Multiplier applied to the axis value
- `offset`: Value added after scaling
- `deadzone`: Values below this threshold are treated as zero

#### Button Mapping Fields
- `button`: Index of the joystick button (0-based)
- `action`: Action to perform when button is pressed
  - `type`: Action type (currently only `publish_twist` supported)
  - Additional fields depend on action type

### Multiple Profiles

You can define multiple profiles in a single file and switch between them at runtime.

## Example Files

- `default.yaml`: General-purpose configuration with multiple profile examples
- `examples/ps4_controller.yaml`: PlayStation 4 controller specific configuration
- `examples/xbox_controller.yaml`: Xbox controller specific configuration

## Usage

To use a configuration file, specify it when launching the node:

```bash
ros2 run joy_msg_router_rs joy_msg_router --ros-args -p config_file:=config/default.yaml
```

Or set the profile at runtime:

```bash
ros2 param set /joy_msg_router profile teleop_safe
```